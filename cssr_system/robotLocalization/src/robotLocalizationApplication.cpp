/* robotLocalizationApplication.cpp
 *
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

// Constructor implementation
RobotLocalizationNode::RobotLocalizationNode() : nh_("~"), it_(nh_), tf_buffer_(), tf_listener_(tf_buffer_) {
    // Load parameters
    nh_.param("verbose", verbose_, false);
    nh_.param("use_depth", use_depth_, false);
    nh_.param("use_head_yaw", use_head_yaw_, false);
    nh_.param("head_yaw_joint_name", head_yaw_joint_name_, std::string("HeadYaw"));
    nh_.param("reset_interval", reset_interval_, 30.0);
    nh_.param("absolute_pose_timeout", absolute_pose_timeout_, 300.0);
    nh_.param("landmark_file", landmark_file_, std::string("data/landmarks.json"));
    nh_.param("topics_file", topics_file_, std::string("data/pepperTopics.dat"));
    nh_.param("camera_info_file", camera_info_file_, std::string("config/camera_info.json"));
    nh_.param("camera_info_timeout", camera_info_timeout_, 10.0);
    nh_.param("map_frame", map_frame_, std::string("map"));
    nh_.param("odom_frame", odom_frame_, std::string("odom"));

    nh_.param("enable_head_scanning", enable_head_scanning_, true);
    nh_.param("use_interval_timer", use_interval_timer_, false);
    nh_.param("perform_startup_localization", perform_startup_localization_, true);
    nh_.param("head_scan_range", head_scan_range_, 1.57);  // 90 degrees in radians
    nh_.param("head_scan_step", head_scan_step_, 0.52);    // 30 degrees in radians
    nh_.param("head_scan_speed", head_scan_speed_, 0.5);
    nh_.param("head_scan_timeout", head_scan_timeout_, 30.0);
    nh_.param("marker_detection_threshold", marker_detection_threshold_, 3.0);

    // Load topic names
    loadTopicNames();

    // Load landmark coordinates
    loadLandmarks();

    if (enable_head_scanning_) {
        initializeHeadScanPositions();
    }

    // Subscribers
    odom_sub_ = nh_.subscribe(topic_map_["Odometry"], 10, &RobotLocalizationNode::odomCallback, this);
    imu_sub_ = nh_.subscribe(topic_map_["IMU"], 10, &RobotLocalizationNode::imuCallback, this);
    image_sub_ = it_.subscribe(topic_map_["RGBRealSense"], 10, &RobotLocalizationNode::imageCallback, this);
    depth_sub_ = it_.subscribe(topic_map_["DepthRealSense"], 10, &RobotLocalizationNode::depthCallback, this);
    joint_sub_ = nh_.subscribe(topic_map_["HeadYaw"], 10, &RobotLocalizationNode::jointCallback, this);
    camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &RobotLocalizationNode::cameraInfoCallback, this);

    // Publishers
    pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
    image_pub_ = it_.advertise("/robotLocalization/marker_image", 10);
    localization_status_pub_ = nh_.advertise<std_msgs::String>("/robotLocalization/status", 10);

    if (enable_head_scanning_) {
        head_yaw_pub_ = nh_.advertise<std_msgs::Float64>("/pepper_dcm/HeadYaw_position_controller/command", 10);
    }

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

    // Status variables
    camera_info_received_ = false;
    first_odom_received_ = false;
    initial_localization_complete_ = false;

    // Sensor variables
    head_yaw_ = 0.0;
    original_head_yaw_ = 0.0;
    camera_height_ = 1.225;
    fx_ = 0.0; fy_ = 0.0; cx_ = 0.0; cy_ = 0.0;

    // Odometry variables
    initial_robot_x = 0.0; initial_robot_y = 0.0; initial_robot_theta = 0.0;
    adjustment_x_ = 0.0; adjustment_y_ = 0.0; adjustment_theta_ = 0.0;
    odom_x_ = 0.0; odom_y_ = 0.0; odom_theta_ = 0.0;

    // Head scanning variables
    head_scan_state_ = HeadScanState::IDLE;
    current_scan_index_ = 0;
    head_movement_complete_ = false;
    target_head_yaw_ = 0.0;
    clearAccumulatedMarkers();

    // Interval timer for periodic reset
    if (use_interval_timer_) {
        reset_timer_ = nh_.createTimer(ros::Duration(reset_interval_), &RobotLocalizationNode::resetTimerCallback, this);
        ROS_INFO("Interval-based localization enabled with %.1f second intervals", reset_interval_);
    }

    // Timer for camera info timeout
    camera_info_timer_ = nh_.createTimer(ros::Duration(camera_info_timeout_), &RobotLocalizationNode::cameraInfoTimeoutCallback, this, true);

    ROS_INFO("=== Robot Localization Configuration ===");
    ROS_INFO("Startup localization: %s", perform_startup_localization_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Interval timer: %s (%.1fs)", use_interval_timer_ ? "ENABLED" : "DISABLED", reset_interval_);
    ROS_INFO("Head scanning: %s", enable_head_scanning_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Service-based localization: ENABLED (call from behavior tree)");
    
    if (enable_head_scanning_) {
        ROS_INFO("  Scan range: ±%.1f degrees", head_scan_range_ * 180.0/M_PI);
        ROS_INFO("  Scan step: %.1f degrees", head_scan_step_ * 180.0/M_PI);
        ROS_INFO("  Scan timeout: %.1f seconds", head_scan_timeout_);
        ROS_INFO("  Marker accumulation: ENABLED");
    }
    ROS_INFO("================================================");
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