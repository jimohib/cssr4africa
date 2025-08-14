#ifndef PEPPER_LOCALIZATION_H
#define PEPPER_LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>

struct MarkerInfo {
    cv::Vec3d position;    // x, y, z in world frame
    cv::Vec3d rotation;    // roll, pitch, yaw in world frame
};

struct RobotPose {
    double x, y, yaw;
    bool valid;
};

class PepperArucoLocalizer {
public:
    PepperArucoLocalizer();
    ~PepperArucoLocalizer();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // ROS components
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pose_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // ArUco detection
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double marker_size_;
    
    // Known marker positions
    std::map<int, MarkerInfo> known_markers_;
    
    // Camera offset from robot base
    cv::Vec3d camera_offset_;
    
    // Coordinate frames
    std::string camera_frame_;
    std::string base_frame_;
    std::string world_frame_;

    RobotPose last_valid_pose_;
    bool has_last_pose_;
    double filter_alpha_;  // 0.0 = no filtering, 1.0 = no smoothing

    
    // Callbacks and methods
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    bool detectMarkers(const cv::Mat& image, std::vector<int>& ids, 
                      std::vector<std::vector<cv::Point2f>>& corners);
    double computeYawFromMarker(const cv::Vec3d& rvec, const cv::Vec3d& tvec, 
                               const MarkerInfo& marker_info);
    RobotPose computeRobotPose(const std::vector<cv::Vec3d>& rvecs,
                              const std::vector<cv::Vec3d>& tvecs,
                              const std::vector<int>& ids);
    cv::Mat eulerToRotationMatrix(const cv::Vec3d& euler);
    void publishPose(double yaw, const std_msgs::Header& header);
    void publishFullPose(const RobotPose& pose, const std_msgs::Header& header);
    void broadcastTransform(double yaw, const std_msgs::Header& header);
    void broadcastFullTransform(const RobotPose& pose, const std_msgs::Header& header);
    void loadMarkerPositions();
    void loadCameraParameters();
};

#endif