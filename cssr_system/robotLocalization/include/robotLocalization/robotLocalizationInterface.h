#ifndef ROBOT_LOCALIZATION_INTERFACE_H
#define ROBOT_LOCALIZATION_INTERFACE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>
#include "cssr_system/SetPose.h"
#include <opencv2/aruco.hpp>

#define ROS_PACKAGE_NAME "cssr_system"

class RobotLocalization {
public:
    RobotLocalization();
    void setInitialValues(double x, double y, double theta);

private:
    void loadConfiguration();
    void loadTopicNames();
    void initROS();
    void initializePoseAdjustments();
    void performAbsolutePoseEstimation();
    void recognizeLandmarks(const cv::Mat& image);
    void publishPose();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool resetPoseService(cssr_system::SetPose::Request& req, cssr_system::SetPose::Response& res);

    std::string trim(const std::string& str);

    // ROS Components
    ros::NodeHandle nh;
    ros::Subscriber odom_sub, imu_sub, joint_state_sub;
    image_transport::Subscriber camera_sub;
    ros::Publisher pose_pub_;
    ros::ServiceServer reset_service;

    //Aruco Components
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Mat camera_matrix_, dist_coeffs_;
    double marker_size_ = 0.2;
    double estimated_pose_x_, estimated_pose_y_, estimated_pose_theta_;
    
    std::map<int, cv::Point3d> marker_positions;

    ros::Time last_update_time_;
    double last_x_, last_y_, last_theta_;
    double movement_threshold_;
    ros::Duration update_interval_;

    // Reset Pose Components
    cssr_system::SetPose::Request req;
    cssr_system::SetPose::Response res;


    // Member Variables
    geometry_msgs::Pose2D relative_pose, last_reset_pose;
    nav_msgs::Odometry previous_odom_;

    double head_yaw_angle_;
    bool process_image, first_odom_received_;

    std::map<std::string, std::string> config_;
    std::map<std::string, std::string> topics_;
    std::string platform_, camera_, robot_topics_file_, simulator_topics_file_;
    int reset_interval_;
    bool verbose_mode;

    cv::Mat current_image_;
    cv_bridge::CvImagePtr cv_ptr_;

    double initial_robot_x, initial_robot_y, initial_robot_theta;
    double adjustment_x_, adjustment_y_, adjustment_theta_;
    double odom_x_, odom_y_, odom_theta_;
};

#endif





// #ifndef ROBOT_LOCALIZATION_INTERFACE_H
// #define ROBOT_LOCALIZATION_INTERFACE_H

// #include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <cssr_system/SetPose.h>

// #define ROS_PACKAGE_NAME "cssr_system"

// class RobotLocalization {
// public:
//     RobotLocalization();
//     void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
//     void arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
//     bool resetPoseService(cssr_system::SetPose::Request &req, cssr_system::SetPose::Response &res);
//     void updateLocalization();

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber odom_sub_, aruco_sub_;
//     ros::Publisher pose_pub_;
//     ros::ServiceServer reset_pose_service_;
//     tf::TransformBroadcaster tf_broadcaster_;

//     // Pose estimation
//     double odom_x_, odom_y_, odom_theta_;
//     double aruco_x_, aruco_y_, aruco_theta_;
//     bool aruco_detected_;

//     // Transformation Matrices
//     tf::Transform current_pose_;

//     void publishPose();
// };

// #endif // ROBOT_LOCALIZATION_INTERFACE_H
