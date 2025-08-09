/* robotLocalizationApplication.cpp    Program initialization and main function execution
 *
 * Author: Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
 * Email: ioj@andrew.cmu.edu
 * Date: June 25, 2025
 * Version: v1.0
 *
 * Copyright (C) 2025 CSSR4Africa Consortium
 *
 * This project is funded by the African Engineering and Technology Network (Afretec)
 * Inclusive Digital Transformation Research Grant Programme.
 *
 * Website: www.cssr4africa.org
 *
 * This program comes with ABSOLUTELY NO WARRANTY.

 * This node is responsible for determining the robot's absolute position and orientation in the environment using visual landmark detection and sensor fusion.
 * The node combines ArUco marker detection from RGB and depth cameras with odometry data to provide accurate 6-DOF pose estimation.
 * The system uses triangulation (RGB-only) or trilateration (with depth) algorithms to compute absolute poses from detected landmarks, then maintains relative positioning through odometry integration.
 * The node supports both periodic automatic pose correction and on-demand pose reset services for robust localization in dynamic environments.
 *
 * Libraries
    * Standard libraries - std::string, std::vector, std::map, std::fstream, std::algorithm, std::numeric, std::sqrt, std::abs, std::atan2, std::cos, std::sin
    * ROS libraries - ros/ros.h, nav_msgs/Odometry.h, sensor_msgs/Imu.h, sensor_msgs/Image.h, sensor_msgs/JointState.h, sensor_msgs/CameraInfo.h, geometry_msgs/Pose2D.h, geometry_msgs/TransformStamped.h
    * ROS TF libraries - tf2/LinearMath/Quaternion.h, tf2_geometry_msgs/tf2_geometry_msgs.h, tf2_ros/transform_listener.h, tf2_ros/buffer.h
    * OpenCV libraries - opencv2/opencv.hpp, opencv2/aruco.hpp, cv_bridge/cv_bridge.h
    * Image transport - image_transport/image_transport.h
    * Utility libraries - angles/angles.h, yaml-cpp/yaml.h
    * 
 * Parameters
    *
    * Command-line Parameters
    *
        * None
    *
    * Configuration File Parameters
        * Key                   |     Value 
        * --------------------- |     -------------------
        * verbose               |     false
        * use_depth             |     false
        * use_head_yaw          |     false
        * head_yaw_joint_name   |     HeadYaw
        * reset_interval        |     30.0
        * absolute_pose_timeout |     300.0
        * landmark_file           |     config/landmarks.yaml
        * topics_file           |     data/pepperTopics.dat
        * camera_info_file      |     config/camera_info.yaml
        * camera_info_timeout   |     10.0
        * map_frame             |     map
        * odom_frame            |     odom
        *
 * Subscribed Topics and Message Types
    *
    * /pepper_dcm/odom                                              nav_msgs/Odometry
    * /pepper_dcm/imu                                               sensor_msgs/Imu
    * /camera/color/image_raw                                       sensor_msgs/Image
    * /camera/depth/image_raw                                       sensor_msgs/Image
    * /joint_states                                                 sensor_msgs/JointState
    * /camera/color/camera_info                                     sensor_msgs/CameraInfo
    *
 * Published Topics and Message Types
    * 
    * /robotLocalization/pose                                       geometry_msgs/Pose2D
    * /robotLocalization/marker_image                               sensor_msgs/Image
    *
 * Services Invoked
    *
    * None
    *
 * Services Advertised and Message Types
    *
    * /robotLocalization/reset_pose                                 cssr_system/ResetPose
    * /robotLocalization/set_pose                                   cssr_system/SetPose
    *
 * Input Data Files
    *
    * pepperTopics.dat - Contains topic names for robot sensors and actuators
    * landmarks.yaml - 3D coordinates of ArUco markers in the environment
    * camera_info.yaml - Camera intrinsic parameters for fallback (fx, fy, cx, cy)
    *
 * Output Data Files
    *
    * None (publishes pose data via ROS topics)
    *
 * Configuration Files
    *
    * landmarks.yaml - Landmark configuration with marker IDs and 3D positions
    * camera_info.yaml - Camera calibration parameters
    * pepperTopics.dat - Topic mapping configuration
    *
 * Example Instantiation of the Module
    *
    * roslaunch cssr_system robotLocalizationLaunchRobot.launch
    *
 * Key Algorithms
    *
    * Triangulation Algorithm (RGB-only mode):
    * - Detects ArUco markers in camera image
    * - Computes viewing angles between marker pairs using camera intrinsics
    * - Uses geometric triangulation with circle-circle intersection to determine robot position
    * - Employs multi-solution scoring system based on geometric constraints
    *
    * Trilateration Algorithm (Depth mode):
    * - Combines ArUco detection with depth measurements
    * - Solves system of distance equations to determine robot position
    * - Uses least-squares approach for overdetermined systems
    *
 * Pose Fusion:
 * - Maintains baseline pose from absolute measurements
 * - Integrates odometry for continuous pose updates
 * - Applies periodic corrections to prevent drift accumulation
 *
 * Marker Detection Pipeline:
 * - Sorts detected markers by position or ID for consistent processing
 * - Validates marker configurations for geometric feasibility
 * - Rejects degenerate cases (collinear markers, extreme angles)
 * - Publishes annotated images showing detected markers
 *
...
*
* Author: Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email: ioj@andrew.cmu.edu
* Date: June 25, 2025
* Version: v1.0
*
*/

#include "robotLocalization/robotLocalizationInterface.h"

// Constructor
RobotLocalizationNode::RobotLocalizationNode() : nh_("~"), it_(nh_), tf_buffer_(), tf_listener_(tf_buffer_) {
    
    // Dynamically resolve config file path in the package
    std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string config_path = package_path + "/robotLocalization/config/robotLocalizationConfiguration.json";

    // Load and parse the JSON configuration file
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        ROS_ERROR("Could not open JSON config file at: %s", config_path.c_str());
        ros::shutdown();
        return;
    }

    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(config_file, config)) {
        ROS_ERROR("Failed to parse JSON config file");
        ros::shutdown();
        return;
    }

    // Load configuration parameters
    verbose_ = config.get("verboseMode", false).asBool();
    camera_ = config.get("camera", "FrontCamera").asString();
    depth_camera_ = config.get("depthCamera", "DepthRealSense").asString();
    use_depth_ = config.get("useDepth", false).asBool();
    reset_interval_ = config.get("resetInterval", 30.0).asDouble();
    absolute_pose_timeout_ = config.get("absolutePoseTimeout", 300.0).asDouble();
    camera_info_timeout_ = config.get("cameraInfoTimeout", 10.0).asDouble();
    use_head_yaw_ = config.get("useHeadYaw", true).asBool();
    head_yaw_joint_name_ = config.get("headYawJointName", "HeadYaw").asString();
    map_frame_ = config.get("mapFrame", "map").asString();
    odom_frame_ = config.get("odomFrame", "odom").asString();
    landmark_file_ = package_path + config.get("landmarkFile", "").asString();
    topics_file_ = package_path + config.get("topicsFile", "").asString();
    camera_info_file_ = package_path + config.get("cameraInfoFile", "").asString();

    // Load topics and landmarks
    loadTopicNames();
    loadLandmarks();

    // Subscribers
    odom_sub_ = nh_.subscribe(topic_map_["Odometry"], 10, &RobotLocalizationNode::odomCallback, this);
    imu_sub_ = nh_.subscribe(topic_map_["IMU"], 10, &RobotLocalizationNode::imuCallback, this);
    image_sub_ = it_.subscribe(topic_map_[camera_], 10, &RobotLocalizationNode::imageCallback, this);
    depth_sub_ = it_.subscribe(topic_map_[depth_camera_], 10, &RobotLocalizationNode::depthCallback, this);
    joint_sub_ = nh_.subscribe(topic_map_["HeadYaw"], 10, &RobotLocalizationNode::jointCallback, this);
    camera_info_sub_ = nh_.subscribe(topic_map_["CameraInfo"], 1, &RobotLocalizationNode::cameraInfoCallback, this);

    // Publishers
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
    image_pub_ = it_.advertise("/robotLocalization/marker_image", 10);

    // Service
    reset_srv_ = nh_.advertiseService("/robotLocalization/reset_pose", &RobotLocalizationNode::resetPoseCallback, this);
    setpose_srv_ = nh_.advertiseService("/robotLocalization/set_pose", &RobotLocalizationNode::setPoseCallback, this);

    initializePoseAdjustments();

    // Initialize pose
    current_pose_.x = 0.0;
    current_pose_.y = 0.0;
    current_pose_.theta = 0.0;
    baseline_pose_ = current_pose_;
    last_odom_pose_ = current_pose_;
    last_absolute_pose_time_ = ros::Time(0);

    // Initialize other variables
    camera_info_received_ = false;
    first_odom_received_ = false;
    head_yaw_ = 0.0;
    camera_height_ = 1.225;
    fx_ = 0.0; fy_ = 0.0; cx_ = 0.0; cy_ = 0.0;
    initial_robot_x = 0.0; initial_robot_y = 0.0; initial_robot_theta = 0.0;
    adjustment_x_ = 0.0; adjustment_y_ = 0.0; adjustment_theta_ = 0.0;
    odom_x_ = 0.0; odom_y_ = 0.0; odom_theta_ = 0.0;

    // Timer for periodic reset
    reset_timer_ = nh_.createTimer(ros::Duration(reset_interval_), &RobotLocalizationNode::resetTimerCallback, this);

    // Timer for camera info timeout
    camera_info_timer_ = nh_.createTimer(ros::Duration(camera_info_timeout_), &RobotLocalizationNode::cameraInfoTimeoutCallback, this, true);

    ROS_INFO("Robot Localization Node initialized");
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "robotLocalization");
    
    // Create the robot localization node
    RobotLocalizationNode node;
    
    // Spin to process callbacks
    ros::spin();
    
    // Clean up OpenCV windows if created
    cv::destroyAllWindows();
    
    return 0;
}