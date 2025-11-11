/* robotLocalizationInterface.h      Function, class, and variable declarations
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
*/

#ifndef ROBOT_LOCALIZATION_INTERFACE_H
#define ROBOT_LOCALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <json/json.h>
#include <fstream>
#include <map>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cssr_system/resetPose.h>
#include <cssr_system/setPose.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#define ROS_PACKAGE_NAME  "cssr_system"
#define SOFTWARE_VERSION  "v1.0"


// Structure for 3D landmark representation
struct Landmark3D {
    int id;
    double x, y, z;
};

// Structure for detected marker with view information
struct DetectedMarker {
    int id;
    std::vector<cv::Point2f> corners;
    double head_yaw;
    ros::Time timestamp;
    std::pair<double, double> center;  // Pre-computed center for efficiency
};

// Robot localization class
class RobotLocalizationNode {
public:
    // Constructor
    RobotLocalizationNode();

private:
    // ROS components
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Subscribers
    ros::Subscriber odom_sub_, imu_sub_, joint_sub_, camera_info_sub_;
    image_transport::Subscriber image_sub_, depth_sub_;
    
    // Publishers
    ros::Publisher pose_pub_;
    image_transport::Publisher image_pub_;
    
    // Services
    ros::ServiceServer reset_srv_, setpose_srv_;
    
    // Timers
    ros::Timer reset_timer_, camera_info_timer_, heartbeat_timer_;

    // Configuration parameters
    bool verbose_, use_depth_, use_head_yaw_, camera_info_received_;
    bool enable_active_scanning_;
    double reset_interval_, camera_info_timeout_, absolute_pose_timeout_;
    double scan_timeout_, marker_memory_timeout_;
    std::string camera_, depth_camera_, head_yaw_joint_name_, map_frame_, odom_frame_;
    std::string landmark_file_, topics_file_, camera_info_file_;
    std::vector<double> scan_positions_;
    
    // Pose tracking variables
    geometry_msgs::Pose2D current_pose_, baseline_pose_, last_odom_pose_;
    ros::Time last_absolute_pose_time_;
    
    // Landmark and topic management
    std::map<int, Landmark3D> landmarks3D_;
    std::map<int, std::pair<double, double>> projected_landmarks_;
    std::map<std::string, std::string> topic_map_;
    
    // Sensor data
    cv::Mat latest_image_, latest_depth_;
    double head_yaw_;
    double camera_height_;
    double fx_, fy_, cx_, cy_; // Camera intrinsics

    // Active scanning components
    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> HeadControlClient;
    boost::shared_ptr<HeadControlClient> head_control_client_;
    std::vector<DetectedMarker> marker_memory_;
    bool is_scanning_;
    double initial_head_yaw_;  // Store initial head position for restoration
    
    // Odometry adjustment variables
    geometry_msgs::Pose2D relative_pose, last_reset_pose;
    nav_msgs::Odometry previous_odom_;
    bool first_odom_received_;
    double initial_robot_x, initial_robot_y, initial_robot_theta;
    double adjustment_x_, adjustment_y_, adjustment_theta_;
    double odom_x_, odom_y_, odom_theta_;

    // Initialization methods
    void initializePoseAdjustments();
    void loadTopicNames();
    void loadLandmarks();
    void loadCameraInfoFromFile();

    // Callback methods
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void cameraInfoTimeoutCallback(const ros::TimerEvent& event);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void resetTimerCallback(const ros::TimerEvent& event);

    // Service callbacks
    bool setPoseCallback(cssr_system::setPose::Request& req, cssr_system::setPose::Response& res);
    bool resetPoseCallback(cssr_system::resetPose::Request& req, cssr_system::resetPose::Response& res);

    // Absolute localization methods
    bool computeAbsolutePose();
    bool computeAbsolutePoseWithDepth();

    // Active scanning methods
    bool computeAbsolutePoseWithActiveScanning();
    bool moveHeadToPosition(double yaw, double pitch = 0.0);
    void detectAndStoreMarkers(double current_head_yaw);
    std::vector<DetectedMarker> selectBestMarkers(int count);
    void cleanupOldMarkers();
    double computeAngleWithHeadYaw(const DetectedMarker& m1, const DetectedMarker& m2);

    // Utility methods
    double computeAngle(const std::pair<double, double>& center1, const std::pair<double, double>& center2);
    double computeYaw(const std::pair<double, double>& marker_center, double marker_x, double marker_y, double robot_x, double robot_y);
    int circle_centre(double x1, double y1, double x2, double y2, double alpha, double *xc1, double *yc1, double *xc2, double *yc2, double *r);
    int circle_circle_intersection(double x0, double y0, double r0, double x1, double y1, double r1, double *xi, double *yi, double *xi_prime, double *yi_prime);
    void publishPose();
};

#endif // ROBOT_LOCALIZATION_H