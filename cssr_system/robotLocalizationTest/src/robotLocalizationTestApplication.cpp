/* robotLocalizationTestApplication.cpp  Program initialization and main function execution for the robot localization unit test
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

 * This node is responsible for running comprehensive unit tests on the robot localization module.
 * The tests validate the communication, computation, and configuration functionality of the components
 * across three main test categories: Test A (Communication), Test B (Computation), and Test C (Configuration).
 * It integrates with both physical robot sensors and test harness environments for controlled testing.
 *
 * Libraries
    * Standard libraries - std::string, std::vector, std::map, std::fstream, std::algorithm, std::numeric
    * ROS libraries - ros/ros.h, ros/package.h, nav_msgs/Odometry.h, sensor_msgs/Imu.h, sensor_msgs/Image.h
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
        * verboseMode           |     true
        * unitTestMode          |     true
        * camera                |     RGBRealSense
        * depthCamera           |     DepthRealSense
        * useDepth              |     false
        * testDuration          |     300.0
        * resetInterval         |     30.0
        * absolutePoseTimeout   |     300.0
        * cameraInfoTimeout     |     15.0
        * useHeadYaw            |     true
        * headYawJointName      |     HeadYaw
        * mapFrame              |     map
        * odomFrame             |     odom
        * landmarkFile          |     /robotLocalizationTest/data/arucoLandmarksTest.json
        * topicsFile            |     /robotLocalizationTest/data/pepperTopicsTest.dat
        * cameraInfoFile        |     /robotLocalizationTest/data/cameraInfoTest.json
    *
 * Subscribed Topics and Message Types
    *
    * /naoqi_driver/odom                                            nav_msgs/Odometry
    * /naoqi_driver/imu/base                                        sensor_msgs/Imu
    * /camera/color/image_raw                                       sensor_msgs/Image
    * /camera/aligned_depth_to_color/image_raw                      sensor_msgs/Image
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
    * /robotLocalization/reset_pose                                 cssr_system/resetPose
    * /robotLocalization/set_pose                                   cssr_system/setPose
    *
 * Input Data Files
    *
    * pepperTopicsTest.dat - Contains topic names for robot sensors and actuators
    * arucoLandmarksTest.json - 3D coordinates of ArUco markers in the test environment
    * cameraInfoTest.json - Camera intrinsic parameters for fallback (fx, fy, cx, cy)
    * harnessTopicsTest.dat - Contains topic names for the test harness environment
    *
 * Output Data Files
    *
    * robotLocalizationTestOutput.dat - Test results and performance metrics
    *
 * Configuration Files
    *
    * robotLocalizationTestConfiguration.json - Main configuration parameters file
    *
 * Example Instantiation of the Module
    *
    * rosrun unit_tests robotLocalizationTest
    *
    * For the launch files:
    *   Physical Robot: roslaunch unit_tests robotLocalizationTestLaunchRobot.launch
    *   Test Harness:   roslaunch unit_tests robotLocalizationTestLaunchHarness.launch
    *
...
*
* Author: Ibrahim Olaide Jimoh, Carnegie Mellon University Africa
* Email: ioj@andrew.cmu.edu
* Date: June 25, 2025
* Version: v1.0
*
*/

#include "robotLocalizationTest/robotLocalizationTestInterface.h"

// Constructor
RobotLocalizationNode::RobotLocalizationNode() : nh_("~"), it_(nh_), tf_buffer_(), tf_listener_(tf_buffer_) {
    
   // Print copyright message
    printCopyrightMessage();
    
    // Record start time
    node_start_time_ = ros::Time::now();
    
    // Print startup message
    printStartupMessage("start-up");

    // Dynamically resolve config file path in the package
    std::string package_path = ros::package::getPath(ROS_PACKAGE_NAME);
    std::string config_path = package_path + "/robotLocalizationTest/config/robotLocalizationTestConfiguration.json";

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
    unit_test_mode_ = config.get("unitTestMode", false).asBool();
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
    printStartupMessage("subscribed to " + topic_map_["Odometry"]);

    imu_sub_ = nh_.subscribe(topic_map_["IMU"], 10, &RobotLocalizationNode::imuCallback, this);
    printStartupMessage("subscribed to " + topic_map_["IMU"]);

    image_sub_ = it_.subscribe(topic_map_[camera_], 10, &RobotLocalizationNode::imageCallback, this);
    printStartupMessage("subscribed to " + topic_map_[camera_]);

    depth_sub_ = it_.subscribe(topic_map_[depth_camera_], 10, &RobotLocalizationNode::depthCallback, this);
    printStartupMessage("subscribed to " + topic_map_[depth_camera_]);

    joint_sub_ = nh_.subscribe(topic_map_["HeadYaw"], 10, &RobotLocalizationNode::jointCallback, this);
    printStartupMessage("subscribed to " + topic_map_["HeadYaw"]);
    
    camera_info_sub_ = nh_.subscribe(topic_map_["CameraInfo"], 1, &RobotLocalizationNode::cameraInfoCallback, this);
    printStartupMessage("subscribed to " + topic_map_["CameraInfo"]);

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

    if (unit_test_mode_) {
        heartbeat_timer_ = nh_.createTimer(ros::Duration(10.0), 
                                         &RobotLocalizationNode::heartbeatCallback, this);
    }

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