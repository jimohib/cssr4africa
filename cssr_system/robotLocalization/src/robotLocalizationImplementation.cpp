#include <ros/ros.h>
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
#include <fstream>
#include <map>
#include <cssr_system/ResetPose.h>
#include <cssr_system/SetPose.h>
#include "robotLocalization/robotLocalizationInterface.h"

struct Landmark3D {
    int id;
    double x, y, z;
};

class RobotLocalizationNode {
public:
    RobotLocalizationNode() : nh_("~"), it_(nh_), tf_buffer_(), tf_listener_(tf_buffer_) {
        // Load parameters
        nh_.param("verbose", verbose_, false);
        nh_.param("use_depth", use_depth_, false);
        nh_.param("use_head_yaw", use_head_yaw_, false);
        nh_.param("head_yaw_joint_name", head_yaw_joint_name_, std::string("HeadYaw"));
        nh_.param("reset_interval", reset_interval_, 30.0);
        nh_.param("absolute_pose_timeout", absolute_pose_timeout_, 300.0);
        nh_.param("config_file", config_file_, std::string("config/landmarks.yaml"));
        nh_.param("topics_file", topics_file_, std::string("data/pepperTopics.dat"));
        nh_.param("camera_info_file", camera_info_file_, std::string("config/camera_info.yaml"));
        nh_.param("camera_info_timeout", camera_info_timeout_, 10.0); // Timeout in seconds
        nh_.param("map_frame", map_frame_, std::string("map"));
        nh_.param("odom_frame", odom_frame_, std::string("odom"));

        // Load topic names
        loadTopicNames();

        // Load landmark coordinates
        loadLandmarks();

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

        // Timer for periodic reset
        reset_timer_ = nh_.createTimer(ros::Duration(reset_interval_), &RobotLocalizationNode::resetTimerCallback, this);

        // Timer for camera info timeout
        camera_info_timer_ = nh_.createTimer(ros::Duration(camera_info_timeout_), &RobotLocalizationNode::cameraInfoTimeoutCallback, this, true);

        ROS_INFO("Robot Localization Node initialized");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber odom_sub_, imu_sub_, joint_sub_, camera_info_sub_;
    image_transport::Subscriber image_sub_, depth_sub_;
    ros::Publisher pose_pub_;
    image_transport::Publisher image_pub_;
    ros::ServiceServer reset_srv_, setpose_srv_;
    ros::Timer reset_timer_, camera_info_timer_;

    bool verbose_, use_depth_, use_head_yaw_, camera_info_received_ = false;
    double reset_interval_, camera_info_timeout_, absolute_pose_timeout_;
    std::string config_file_, topics_file_, camera_info_file_;
    std::string head_yaw_joint_name_, map_frame_, odom_frame_;
    geometry_msgs::Pose2D current_pose_, baseline_pose_, last_odom_pose_;
    ros::Time last_absolute_pose_time_;
    std::map<int, Landmark3D> landmarks3D_;
    std::map<int, std::pair<double, double>> projected_landmarks_;
    std::map<std::string, std::string> topic_map_;
    cv::Mat latest_image_, latest_depth_;
    double head_yaw_ = 0.0;
    double camera_height_ = 1.225;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0; // Camera intrinsics initialize


    geometry_msgs::Pose2D relative_pose, last_reset_pose;
    nav_msgs::Odometry previous_odom_;
    bool first_odom_received_ = false;
    double initial_robot_x=0.0, initial_robot_y=0.0, initial_robot_theta=0.0;
    double adjustment_x_=0.0, adjustment_y_=0.0, adjustment_theta_=0.0;
    double odom_x_=0.0, odom_y_=0.0, odom_theta_=0.0;
    


    void initializePoseAdjustments() {
        ros::Rate rate(10);
        while (!first_odom_received_ && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
        adjustment_x_ = initial_robot_x - odom_x_;
        adjustment_y_ = initial_robot_y - odom_y_;
        adjustment_theta_ = angles::normalize_angle(initial_robot_theta - odom_theta_);
    }

    void loadTopicNames() {
        try {
            std::ifstream file(topics_file_);
            if (!file.is_open()) {
                ROS_ERROR("Failed to open topics file %s", topics_file_.c_str());
                return;
            }
            std::string line;
            while (std::getline(file, line)) {
                size_t pos = line.find("=");
                if (pos != std::string::npos) {
                    std::string key = line.substr(0, pos);
                    std::string value = line.substr(pos + 1);
                    key.erase(key.find_last_not_of(" \t") + 1);
                    value.erase(0, value.find_first_not_of(" \t"));
                    topic_map_[key] = value;
                }
            }
            ROS_INFO("Loaded topics from %s", topics_file_.c_str());
        } catch (const std::exception& e) {
            ROS_ERROR("Error reading topics file %s: %s", topics_file_.c_str(), e.what());
        }
    }

    void loadLandmarks() {
        try {
            YAML::Node config = YAML::LoadFile(config_file_);
            if (!config["landmarks"]) {
                ROS_ERROR("No 'landmarks' key found in %s", config_file_.c_str());
                return;
            }
            for (const auto& marker : config["landmarks"]) {
                if (!marker["id"] || !marker["x"] || !marker["y"] || !marker["z"]) {
                    ROS_WARN("Skipping invalid landmark entry in %s", config_file_.c_str());
                    continue;
                }
                Landmark3D lm;
                lm.id = marker["id"].as<int>();
                lm.x = marker["x"].as<double>();
                lm.y = marker["y"].as<double>();
                lm.z = marker["z"] ? marker["z"].as<double>() : camera_height_; // Fallback to camera height

                landmarks3D_[lm.id] = lm;

                // Project to camera plane
                double dz = lm.z - camera_height_;
                // Compute distance in 3D
                double distance = std::sqrt(lm.x * lm.x + lm.y * lm.y + dz * dz);
                if (distance < 0.1) { // Avoid division by zero
                    ROS_WARN("Landmark %d too close to camera, skipping projection", lm.id);
                    continue;
                }
                // Project to 2D plane at camera height
                double scale = std::sqrt(lm.x * lm.x + lm.y * lm.y) / distance;
                double x_proj = lm.x * scale;
                double y_proj = lm.y * scale;

                projected_landmarks_[lm.id] = {x_proj, y_proj};

                if (verbose_) {
                    ROS_INFO("Loaded landmark ID %d: (%.2f, %.2f, %.2f) -> Projected (%.2f, %.2f)",
                             lm.id, lm.x, lm.y, lm.z, x_proj, y_proj);
                }
            }
        } catch (const YAML::BadFile& e) {
            ROS_ERROR("Failed to open landmarks file %s: %s", config_file_.c_str(), e.what());
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Failed to parse landmarks file %s: %s", config_file_.c_str(), e.what());
        }
    }

    void loadCameraInfoFromFile() {
        try {
            YAML::Node config = YAML::LoadFile(camera_info_file_);
            fx_ = config["camera_info"]["fx"].as<double>();
            fy_ = config["camera_info"]["fy"].as<double>();
            cx_ = config["camera_info"]["cx"].as<double>();
            cy_ = config["camera_info"]["cy"].as<double>();
            if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
                ROS_ERROR("Invalid camera intrinsics in %s", camera_info_file_.c_str());
                return;
            }
            camera_info_received_ = true;
            ROS_INFO("Loaded camera intrinsics from file: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Failed to load camera info from %s: %s", camera_info_file_.c_str(), e.what());
        }
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        if (camera_info_received_) return;
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
            ROS_WARN("Invalid camera intrinsics received from topic");
            return;
        }
        camera_info_received_ = true;
        camera_info_timer_.stop();
        ROS_INFO("Received camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
    }

    void cameraInfoTimeoutCallback(const ros::TimerEvent& event) {
        if (!camera_info_received_) {
            ROS_WARN("No camera info received within %.1f seconds, falling back to %s", camera_info_timeout_, camera_info_file_.c_str());
            loadCameraInfoFromFile();
        }
    }

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //     // Transform odometry pose to map frame
    //     geometry_msgs::PoseStamped odom_pose, map_pose;
    //     odom_pose.header = msg->header;
    //     odom_pose.pose = msg->pose.pose;
    //     try {
    //         tf_buffer_.transform(odom_pose, map_pose, map_frame_, ros::Duration(0.1));
    //     } catch (const tf2::TransformException& ex) {
    //         ROS_WARN("Failed to transform odometry pose: %s", ex.what());
    //         return;
    //     }

    //     // Extract x, y, theta from transformed pose
    //     double x = map_pose.pose.position.x;
    //     double y = map_pose.pose.position.y;
    //     tf2::Quaternion q(
    //         map_pose.pose.orientation.x,
    //         map_pose.pose.orientation.y,
    //         map_pose.pose.orientation.z,
    //         map_pose.pose.orientation.w);
    //     tf2::Matrix3x3 m(q);
    //     double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);

    //     // Compute deltas relative to last odometry pose
    //     double delta_x = x - last_odom_pose_.x;
    //     double delta_y = y - last_odom_pose_.y;
    //     double delta_theta = yaw - last_odom_pose_.theta;
    //     while (delta_theta > M_PI) delta_theta -= 2 * M_PI;
    //     while (delta_theta < -M_PI) delta_theta += 2 * M_PI;

    //     // Update last odometry pose
    //     last_odom_pose_.x = x;
    //     last_odom_pose_.y = y;
    //     last_odom_pose_.theta = yaw;

    //     // Apply deltas to baseline pose
    //     if (last_absolute_pose_time_.isValid() && (ros::Time::now() - last_absolute_pose_time_).toSec() < absolute_pose_timeout_) {
    //         // Rotate delta_x, delta_y by baseline theta
    //         double cos_theta = std::cos(baseline_pose_.theta);
    //         double sin_theta = std::sin(baseline_pose_.theta);
    //         current_pose_.x = baseline_pose_.x + delta_x * cos_theta + delta_y * sin_theta;
    //         current_pose_.y = baseline_pose_.y - delta_x * sin_theta + delta_y * cos_theta;
    //         current_pose_.theta = baseline_pose_.theta + delta_theta;
    //         while (current_pose_.theta > M_PI) current_pose_.theta -= 2 * M_PI;
    //         while (current_pose_.theta < -M_PI) current_pose_.theta += 2 * M_PI;
    //     } else {
    //         // Use raw odometry if no recent absolute pose
    //         current_pose_.x = x;
    //         current_pose_.y = y;
    //         current_pose_.theta = yaw;
    //     }

    //     publishPose();
    //     if (verbose_) {
    //         ROS_INFO("Odometry update: x=%.3f, y=%.3f, theta=%.3f degrees, deltas: dx=%.3f, dy=%.3f, dtheta=%.3f degrees",
    //                  current_pose_.x, current_pose_.y, current_pose_.theta * 180.0 / M_PI,
    //                  delta_x, delta_y, delta_theta * 180.0 / M_PI);
    //     }
    // }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_theta_ = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    first_odom_received_ = true;

    double x = odom_x_ + adjustment_x_ - initial_robot_x;
    double y = odom_y_ + adjustment_y_ - initial_robot_y;

    double current_x = x * cos(adjustment_theta_) - y * sin(adjustment_theta_);
    double current_y = x * sin(adjustment_theta_) + y * cos(adjustment_theta_);

    current_x += initial_robot_x;
    current_y += initial_robot_y;

    double current_theta = odom_theta_ + adjustment_theta_;

    relative_pose.x = current_x;
    relative_pose.y = current_y;
    relative_pose.theta = current_theta;

    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = relative_pose.x;
    pose_msg.y = relative_pose.y;
    pose_msg.theta = angles::to_degrees(relative_pose.theta);

    pose_pub_.publish(pose_msg);

    if (verbose_) {
        ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f degrees", current_x, current_y, angles::to_degrees(current_theta));
    }
}

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Optional using IMU angular velocity
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        bool found = false;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == head_yaw_joint_name_) {
                head_yaw_ = msg->position[i]; // Radians
                found = true;
                if (verbose_) {
                    ROS_INFO("Head yaw update: %.3f radians", head_yaw_);
                }
                break;
            }
        }
        if (!found && verbose_) {
            std::string names;
            for (const auto& name : msg->name) names += name + ", ";
            ROS_WARN("Head yaw joint '%s' not found in joint_states: %s", head_yaw_joint_name_.c_str(), names.c_str());
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            if (verbose_) {
                ROS_INFO("Received image: width=%d, height=%d", latest_image_.cols, latest_image_.rows);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            latest_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image;
            if (verbose_) {
                ROS_INFO("Received depth image: width=%d, height=%d", latest_depth_.cols, latest_depth_.rows);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    // bool setPoseCallback(cssr_system::SetPose::Request& req, cssr_system::SetPose::Response& res) {
    //     baseline_pose_.x = req.x;
    //     baseline_pose_.y = req.y;
    //     baseline_pose_.theta = req.theta * M_PI / 180.0; // Convert degrees to radians
    //     while (baseline_pose_.theta > M_PI) baseline_pose_.theta -= 2 * M_PI;
    //     while (baseline_pose_.theta < -M_PI) baseline_pose_.theta += 2 * M_PI;
    //     current_pose_ = baseline_pose_;
    //     last_absolute_pose_time_ = ros::Time::now();
    //     publishPose();
    //     if (verbose_) {
    //         ROS_INFO("Manually set pose: x=%.3f, y=%.3f, theta=%.3f degrees", current_pose_.x, current_pose_.y, req.theta);
    //     }
    //     res.success = true;
    //     return true;
    // }

    bool setPoseCallback(cssr_system::SetPose::Request& req, cssr_system::SetPose::Response& res) {
        initial_robot_x = req.x;
        initial_robot_y = req.y;
        initial_robot_theta = angles::from_degrees(req.theta);

        ros::Rate rate(10);
        while (!first_odom_received_ && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }

        adjustment_x_ = initial_robot_x - odom_x_;
        adjustment_y_ = initial_robot_y - odom_y_;
        adjustment_theta_ = initial_robot_theta - odom_theta_;
        res.success = true;
        return true;
    }

    bool resetPoseCallback(cssr_system::ResetPose::Request& req, cssr_system::ResetPose::Response& res) {
        if (computeAbsolutePose()) {
            res.success = true;
            ROS_INFO("Pose reset successfully");
        } else {
            res.success = false;
            ROS_WARN("Failed to reset pose");
        }
        return true;
    }

    void resetTimerCallback(const ros::TimerEvent& event) {
        computeAbsolutePose();
    }

    bool computeAbsolutePose() {
        if (projected_landmarks_.empty()) {
            ROS_WARN("No landmarks loaded");
            return false;
        }
        if (!camera_info_received_) {
            ROS_WARN("Camera intrinsics not received");
            return false;
        }
        if (use_depth_) {
            return computeAbsolutePoseWithDepth();
        } else {
            if (latest_image_.empty()) {
                ROS_WARN("No image available for absolute pose estimation");
                return false;
            }

            // Detect ArUco markers
            std::vector<int> marker_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
            cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

             // Draw and publish marker image
            cv::Mat output_image = latest_image_.clone();
            if (!marker_ids.empty()) {
                cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
            }
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
            image_pub_.publish(img_msg);
            if (verbose_) {
                ROS_INFO("Published marker image with %zu markers", marker_ids.size());
            }

            if (marker_ids.size() < 3) {
                ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
                return false;
            }

            // // Draw bounding boxes
            // cv::Mat output_image = latest_image_.clone();
            // cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

            // Compute angles and triangulate
            std::vector<std::pair<double, double>> marker_centers;
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                auto& corners = marker_corners[i];
                double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
                double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
                marker_centers.push_back({cx, cy});
            }

            // Assume first three detected markers are used
            int id1 = marker_ids[0], id2 = marker_ids[1], id3 = marker_ids[2];
            if (projected_landmarks_.find(id1) == projected_landmarks_.end() || projected_landmarks_.find(id2) == projected_landmarks_.end() || projected_landmarks_.find(id3) == projected_landmarks_.end()) {
                ROS_WARN("Unknown marker IDs detected: %d, %d, %d", id1, id2, id3);
                return false;
            }

            double x1 = projected_landmarks_[id1].first, y1 = projected_landmarks_[id1].second;
            double x2 = projected_landmarks_[id2].first, y2 = projected_landmarks_[id2].second;
            double x3 = projected_landmarks_[id3].first, y3 = projected_landmarks_[id3].second;

            // Check for collinear markers
            double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            if (std::abs(cross_product) < 0.01) {
                ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
                return false;
            }

            // Compute angles alpha1 (between markers 1 and 2) and alpha2 (between markers 2 and 3)
            double alpha1 = computeAngle(marker_centers[0], marker_centers[1]);
            double alpha2 = computeAngle(marker_centers[1], marker_centers[2]);

            if (verbose_) {
                ROS_INFO("Computed angles: alpha1=%.3f degrees, alpha2=%.3f degrees", alpha1, alpha2);
            }

            // Triangulation
            double xc1a, yc1a, xc1b, yc1b, xc2a, yc2a, xc2b, yc2b, r1, r2;
            double x1_intersection, y1_intersection, x2_intersection, y2_intersection;
            double tolerance = 0.001;

            circle_centre(x2, y2, x1, y1, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
            circle_centre(x3, y3, x2, y2, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);

            int result = circle_circle_intersection(xc1a, yc1a, r1, xc2a, yc2a, r2, &x1_intersection, &y1_intersection, &x2_intersection, &y2_intersection);
            if (result == 0) {
                ROS_WARN("Circles do not intersect");
                return false;
            }

            // Determine robot position
            double xr, yr;
            if ((std::abs(x1_intersection - x1) < tolerance && std::abs(y1_intersection - y1) < tolerance) ||
                (std::abs(x1_intersection - x2) < tolerance && std::abs(y1_intersection - y2) < tolerance) ||
                (std::abs(x1_intersection - x3) < tolerance && std::abs(y1_intersection - y3) < tolerance)) {
                xr = x2_intersection;
                yr = y2_intersection;
            } else {
                xr = x1_intersection;
                yr = y1_intersection;
            }

            // Compute yaw (using marker 1)
            double theta = computeYaw(marker_centers[0], x1, y1, xr, yr);

            // Update pose
            baseline_pose_.x = xr;
            baseline_pose_.y = yr;
            baseline_pose_.theta = theta; // Radians
            current_pose_ = baseline_pose_;
            last_absolute_pose_time_ = ros::Time::now();

            // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
            // image_pub_.publish(img_msg);

            // Alternate update pose
            initial_robot_x = xr;
            initial_robot_y = yr;
            initial_robot_theta = theta;
            adjustment_x_ = initial_robot_x - odom_x_;
            adjustment_y_ = initial_robot_y - odom_y_;
            adjustment_theta_ = initial_robot_theta - odom_theta_;


            ROS_INFO("Robot Pose: x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
            cv::imshow("ArUco Markers", output_image);
            cv::waitKey(1);

            // publishPose();
            return true;
        }
    }

    // bool computeAbsolutePoseWithDepth() {
    //     if (latest_image_.empty() || latest_depth_.empty()) {
    //         ROS_WARN("No image or depth data available");
    //         return false;
    //     }
    
    //     // Detect ArUco markers
    //     std::vector<int> marker_ids;
    //     std::vector<std::vector<cv::Point2f>> marker_corners;
    //     cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    //     cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

    //     // Draw and publish marker image
    //     cv::Mat output_image = latest_image_.clone();
    //     if (!marker_ids.empty()) {
    //         cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    //     }
    //     sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
    //     image_pub_.publish(img_msg);
    //     if (verbose_) {
    //         ROS_INFO("Published marker image with %zu markers", marker_ids.size());
    //     }
    
    //     if (marker_ids.size() < 3) {
    //         ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
    //         return false;
    //     }
    
    //     // Get distances from depth image
    //     std::vector<std::tuple<int, double, double, double>> markers; // id, x, y, distance
    //     for (size_t i = 0; i < marker_ids.size(); ++i) {
    //         auto& corners = marker_corners[i];
    //         double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
    //         double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
    //         float distance = latest_depth_.at<float>(int(cy), int(cx)) / 1000.0; // Convert mm to m
    //         if (projected_landmarks_.find(marker_ids[i]) != projected_landmarks_.end() && !std::isnan(distance)) {
    //             markers.push_back({marker_ids[i], projected_landmarks_[marker_ids[i]].first, projected_landmarks_[marker_ids[i]].second, distance});
    //         }
    //         // ROS_INFO("Marker ID %d: Distance = %.3f m", marker_ids[i], distance);
    //     }

    //     std::sort(markers.begin(), markers.end(), 
    //         [](const std::tuple<int, double, double, double>& a, 
    //         const std::tuple<int, double, double, double>& b) {
    //             return std::get<3>(a) < std::get<3>(b); // Sort by distance (closest first)
    //         });

    //     ROS_INFO("Detected %zu markers:", markers.size());
    //     for (const auto& marker : markers) {
    //         ROS_INFO("  Marker ID %d: Position (%.3f, %.3f), Distance = %.3f m", 
    //                 std::get<0>(marker), std::get<1>(marker), std::get<2>(marker), std::get<3>(marker));
    //     }
    
    //     if (markers.size() < 3) {
    //         ROS_WARN("Insufficient valid depth measurements");
    //         return false;
    //     }
    
    //     // Trilateration: Solve for (xr, yr) using three markers
    //     double x1 = std::get<1>(markers[0]), y1 = std::get<2>(markers[0]), d1 = std::get<3>(markers[0]);
    //     double x2 = std::get<1>(markers[1]), y2 = std::get<2>(markers[1]), d2 = std::get<3>(markers[1]);
    //     double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);

    //      // Check for collinear markers
    //     double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    //     if (std::abs(cross_product) < 0.01) {
    //         ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
    //         return false;
    //     }
    
    //     // Simplified trilateration (intersection of two circles)
    //     double x12 = x2 - x1, y12 = y2 - y1;
    //     double d12 = std::sqrt(x12 * x12 + y12 * y12);
    //     double a = (d1 * d1 - d2 * d2 + d12 * d12) / (2.0 * d12);
    //     double h = std::sqrt(d1 * d1 - a * a);
    //     double xm = x1 + a * x12 / d12;
    //     double ym = y1 + a * y12 / d12;
    
    //     double xr1 = xm + h * (y2 - y1) / d12;
    //     double yr1 = ym - h * (x2 - x1) / d12;
    //     double xr2 = xm - h * (y2 - y1) / d12;
    //     double yr2 = ym + h * (x2 - x1) / d12;
    
    //     // Check which solution satisfies the third circle
    //     double dist1 = std::sqrt((xr1 - x3) * (xr1 - x3) + (yr1 - y3) * (yr1 - y3));
    //     double dist2 = std::sqrt((xr2 - x3) * (xr2 - x3) + (yr2 - y3) * (yr2 - y3));
    //     double xr, yr;
    //     if (std::abs(dist1 - d3) < std::abs(dist2 - d3)) {
    //         xr = xr1;
    //         yr = yr1;
    //     } else {
    //         xr = xr2;
    //         yr = yr2;
    //     }
    
    //     // Compute yaw
    //     double theta = computeYaw({marker_corners[0][0].x, marker_corners[0][0].y}, x1, y1, xr, yr);
    
    //     // Update pose
    //     baseline_pose_.x = xr;
    //     baseline_pose_.y = yr;
    //     baseline_pose_.theta = theta; // Radians
    //     current_pose_ = baseline_pose_;
    //     last_absolute_pose_time_ = ros::Time::now();
    
    //     // // Publish image
    //     // cv::Mat output_image = latest_image_.clone();
    //     // cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
    //     // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
    //     // image_pub_.publish(img_msg);
    
    //     // Alternate update pose
    //     initial_robot_x = xr;
    //     initial_robot_y = yr;
    //     initial_robot_theta = theta;
    //     adjustment_x_ = initial_robot_x - odom_x_;
    //     adjustment_y_ = initial_robot_y - odom_y_;
    //     adjustment_theta_ = initial_robot_theta - odom_theta_;

    //     ROS_INFO("Robot Pose (Depth): x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
    //     cv::imshow("ArUco Markers", output_image);
    //     cv::waitKey(1);

    
    //     // publishPose();
    //     return true;
    // }


     bool computeAbsolutePoseWithDepth() {
        if (latest_image_.empty() || latest_depth_.empty()) {
            ROS_WARN("No image or depth data available");
            return false;
        }

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

        // Draw and publish marker image
        cv::Mat output_image = latest_image_.clone();
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
        }
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
        image_pub_.publish(img_msg);
        if (verbose_) {
            ROS_INFO("Published marker image with %zu markers", marker_ids.size());
        }

        if (marker_ids.size() < 2) {
            ROS_WARN("Detected %zu markers, need at least 2", marker_ids.size());
            return false;
        }

        // Get horizontal distances from depth image (correcting for height differences)
        std::vector<std::tuple<int, double, double, double, cv::Point2f>> markers; // id, x, y, horizontal_distance, image_center
        
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            // Check if this marker is in our landmark database
            if (landmarks3D_.find(marker_ids[i]) == landmarks3D_.end()) {
                ROS_WARN("Unknown marker ID %d, skipping", marker_ids[i]);
                continue;
            }

            auto& corners = marker_corners[i];
            double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
            double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
            
            // Get depth sensor reading (3D distance from camera to marker)
            float depth_distance = latest_depth_.at<float>(int(cy), int(cx)) / 1000.0; // Convert mm to m
            
            if (std::isnan(depth_distance) || depth_distance <= 0.0) {
                ROS_WARN("Invalid depth reading for marker %d", marker_ids[i]);
                continue;
            }

            // Get marker's known 3D position
            const Landmark3D& landmark = landmarks3D_[marker_ids[i]];
            double marker_height = landmark.z;
            
            // Calculate height difference between camera and marker
            double height_diff = marker_height - camera_height_;
            
            // Project 3D depth distance to horizontal (X-Y plane) distance
            // Using Pythagorean theorem: horizontal_distance^2 + height_diff^2 = depth_distance^2
            double horizontal_distance_squared = depth_distance * depth_distance - height_diff * height_diff;
            
            if (horizontal_distance_squared <= 0.0) {
                ROS_WARN("Marker %d: height difference too large for depth reading (%.3f vs %.3f)", 
                        marker_ids[i], depth_distance, std::abs(height_diff));
                continue;
            }
            
            double horizontal_distance = std::sqrt(horizontal_distance_squared);
            
            // Use the projected landmark coordinates (already projected to camera height plane)
            if (projected_landmarks_.find(marker_ids[i]) == projected_landmarks_.end()) {
                ROS_WARN("Marker %d not found in projected landmarks", marker_ids[i]);
                continue;
            }
            
            double proj_x = projected_landmarks_[marker_ids[i]].first;
            double proj_y = projected_landmarks_[marker_ids[i]].second;
            
            markers.push_back({marker_ids[i], proj_x, proj_y, horizontal_distance, cv::Point2f(cx, cy)});
            
            if (verbose_) {
                ROS_INFO("Marker ID %d: 3D distance=%.3f m, height_diff=%.3f m, horizontal_distance=%.3f m", 
                        marker_ids[i], depth_distance, height_diff, horizontal_distance);
            }
        }

        // Sort by distance (closest first) for better numerical stability
        std::sort(markers.begin(), markers.end(), 
            [](const std::tuple<int, double, double, double, cv::Point2f>& a, 
            const std::tuple<int, double, double, double, cv::Point2f>& b) {
                return std::get<3>(a) < std::get<3>(b);
            });

        ROS_INFO("Valid markers for localization: %zu", markers.size());
        for (const auto& marker : markers) {
            ROS_INFO("  Marker ID %d: Position (%.3f, %.3f), Horizontal Distance = %.3f m", 
                    std::get<0>(marker), std::get<1>(marker), std::get<2>(marker), std::get<3>(marker));
        }

        if (markers.size() < 2) {
            ROS_WARN("Insufficient valid markers for localization (need at least 2)");
            return false;
        }

        // Use the two closest/best markers for localization
        double x1 = std::get<1>(markers[0]), y1 = std::get<2>(markers[0]), d1 = std::get<3>(markers[0]);
        double x2 = std::get<1>(markers[1]), y2 = std::get<2>(markers[1]), d2 = std::get<3>(markers[1]);
        cv::Point2f img1 = std::get<4>(markers[0]);
        cv::Point2f img2 = std::get<4>(markers[1]);

        // Check that the two landmarks are sufficiently separated
        double landmark_distance = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        double max_measurement_distance = std::max(d1, d2);
        
        if (landmark_distance < max_measurement_distance * 0.2) {
            ROS_WARN("Landmarks too close together for reliable localization (%.3f m apart, max distance %.3f m)", 
                    landmark_distance, max_measurement_distance);
            // Continue anyway but with reduced confidence
        }

        // Two-circle intersection (bilateration)
        double dx = x2 - x1;
        double dy = y2 - y1;
        double d = std::sqrt(dx * dx + dy * dy);
        
        if (d == 0) {
            ROS_ERROR("Two landmarks are at the same location");
            return false;
        }
        
        // Check if circles can intersect
        if (d > d1 + d2) {
            ROS_WARN("Circles don't intersect - landmarks too far apart relative to distances");
            return false;
        }
        if (d < std::abs(d1 - d2)) {
            ROS_WARN("One circle is contained in the other");
            return false;
        }

        // Calculate intersection points
        double a = (d1 * d1 - d2 * d2 + d * d) / (2.0 * d);
        double h_squared = d1 * d1 - a * a;
        
        if (h_squared < 0) {
            ROS_WARN("No real intersection points");
            return false;
        }
        
        double h = std::sqrt(h_squared);
        
        // Point on line between circle centers
        double x_mid = x1 + a * dx / d;
        double y_mid = y1 + a * dy / d;
        
        // Two possible robot positions
        double xr1 = x_mid + h * dy / d;
        double yr1 = y_mid - h * dx / d;
        double xr2 = x_mid - h * dy / d;
        double yr2 = y_mid + h * dx / d;

        ROS_INFO("Two possible positions: (%.3f, %.3f) and (%.3f, %.3f)", xr1, yr1, xr2, yr2);

        // Choose the correct position using additional information
        double xr, yr;
        
        // First check: Use known map boundaries (X: 0.0-7.0, Y: 0.0-10.0)
        bool pos1_in_bounds = (xr1 >= 0.0 && xr1 <= 7.0 && yr1 >= 0.0 && yr1 <= 10.0);
        bool pos2_in_bounds = (xr2 >= 0.0 && xr2 <= 7.0 && yr2 >= 0.0 && yr2 <= 10.0);
        
        if (pos1_in_bounds && !pos2_in_bounds) {
            xr = xr1; yr = yr1;
            ROS_INFO("Selected position 1 (%.3f, %.3f) - only position within map bounds [0-7, 0-10]", xr1, yr1);
        } else if (pos2_in_bounds && !pos1_in_bounds) {
            xr = xr2; yr = yr2;
            ROS_INFO("Selected position 2 (%.3f, %.3f) - only position within map bounds [0-7, 0-10]", xr2, yr2);
        } else if (!pos1_in_bounds && !pos2_in_bounds) {
            ROS_WARN("Both positions outside map bounds: (%.3f, %.3f) and (%.3f, %.3f)", xr1, yr1, xr2, yr2);
            // Continue with other disambiguation methods
            if (markers.size() >= 3) {
                // Use third marker to disambiguate
                double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);
                double dist1_to_3 = std::sqrt((xr1 - x3) * (xr1 - x3) + (yr1 - y3) * (yr1 - y3));
                double dist2_to_3 = std::sqrt((xr2 - x3) * (xr2 - x3) + (yr2 - y3) * (yr2 - y3));
                double error1 = std::abs(dist1_to_3 - d3);
                double error2 = std::abs(dist2_to_3 - d3);
                
                if (error1 < error2) {
                    xr = xr1; yr = yr1;
                    ROS_INFO("Selected position 1 using third marker (error: %.3f vs %.3f) - both out of bounds", error1, error2);
                } else {
                    xr = xr2; yr = yr2;
                    ROS_INFO("Selected position 2 using third marker (error: %.3f vs %.3f) - both out of bounds", error1, error2);
                }
            } else {
                // Default to position 1 if no other info available
                xr = xr1; yr = yr1;
                ROS_WARN("Both positions out of bounds, defaulting to position 1 (%.3f, %.3f)", xr1, yr1);
            }
        } else if (pos1_in_bounds && pos2_in_bounds) {
            // Both positions are within bounds, use third marker if available
            ROS_INFO("Both positions within bounds: (%.3f, %.3f) and (%.3f, %.3f)", xr1, yr1, xr2, yr2);
            if (markers.size() >= 3) {
                // Use third marker to disambiguate
                double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);
                double dist1_to_3 = std::sqrt((xr1 - x3) * (xr1 - x3) + (yr1 - y3) * (yr1 - y3));
                double dist2_to_3 = std::sqrt((xr2 - x3) * (xr2 - x3) + (yr2 - y3) * (yr2 - y3));
                double error1 = std::abs(dist1_to_3 - d3);
                double error2 = std::abs(dist2_to_3 - d3);
                
                if (error1 < error2) {
                    xr = xr1; yr = yr1;
                    ROS_INFO("Selected position 1 using third marker (error: %.3f vs %.3f)", error1, error2);
                } else {
                    xr = xr2; yr = yr2;
                    ROS_INFO("Selected position 2 using third marker (error: %.3f vs %.3f)", error1, error2);
                }
            } else {
            // Use previous position for disambiguation (temporal consistency)
            if (last_absolute_pose_time_.isValid()) {
                double dist1_to_prev = std::sqrt((xr1 - current_pose_.x) * (xr1 - current_pose_.x) + 
                                            (yr1 - current_pose_.y) * (yr1 - current_pose_.y));
                double dist2_to_prev = std::sqrt((xr2 - current_pose_.x) * (xr2 - current_pose_.x) + 
                                            (yr2 - current_pose_.y) * (yr2 - current_pose_.y));
                
                if (dist1_to_prev < dist2_to_prev) {
                    xr = xr1; yr = yr1;
                    ROS_INFO("Selected position 1 based on previous position (distances: %.3f vs %.3f)", 
                            dist1_to_prev, dist2_to_prev);
                } else {
                    xr = xr2; yr = yr2;
                    ROS_INFO("Selected position 2 based on previous position (distances: %.3f vs %.3f)", 
                            dist1_to_prev, dist2_to_prev);
                }
            } else {
                // Use heading information to disambiguate
                // Compute expected angles to both landmarks from both positions
                double angle1_to_lm1 = atan2(y1 - yr1, x1 - xr1);
                double angle1_to_lm2 = atan2(y2 - yr1, x2 - xr1);
                double angle2_to_lm1 = atan2(y1 - yr2, x1 - xr2);
                double angle2_to_lm2 = atan2(y2 - yr2, x2 - xr2);
                
                // Compute image angles (simplified - using image center as reference)
                double img_angle1 = atan2(img1.y - cy_, img1.x - cx_);
                double img_angle2 = atan2(img2.y - cy_, img2.x - cx_);
                
                // Choose position that better matches the relative angles observed in image
                double relative_world_angle1 = angle1_to_lm2 - angle1_to_lm1;
                double relative_world_angle2 = angle2_to_lm2 - angle2_to_lm1;
                double relative_img_angle = img_angle2 - img_angle1;
                
                // Normalize angles to [-π, π]
                while (relative_world_angle1 > M_PI) relative_world_angle1 -= 2*M_PI;
                while (relative_world_angle1 < -M_PI) relative_world_angle1 += 2*M_PI;
                while (relative_world_angle2 > M_PI) relative_world_angle2 -= 2*M_PI;
                while (relative_world_angle2 < -M_PI) relative_world_angle2 += 2*M_PI;
                while (relative_img_angle > M_PI) relative_img_angle -= 2*M_PI;
                while (relative_img_angle < -M_PI) relative_img_angle += 2*M_PI;
                
                double angle_error1 = std::abs(relative_world_angle1 - relative_img_angle);
                double angle_error2 = std::abs(relative_world_angle2 - relative_img_angle);
                
                if (angle_error1 < angle_error2) {
                    xr = xr1; yr = yr1;
                    ROS_INFO("Selected position 1 based on angle consistency (errors: %.3f vs %.3f rad)", 
                            angle_error1, angle_error2);
                } else {
                    xr = xr2; yr = yr2;
                    ROS_INFO("Selected position 2 based on angle consistency (errors: %.3f vs %.3f rad)", 
                            angle_error1, angle_error2);
                }
            }
        }
    }

        // Compute yaw using the first marker
        double theta = computeYaw({img1.x, img1.y}, x1, y1, xr, yr);

        // Validate the solution
        double verification_d1 = std::sqrt((xr - x1) * (xr - x1) + (yr - y1) * (yr - y1));
        double verification_d2 = std::sqrt((xr - x2) * (xr - x2) + (yr - y2) * (yr - y2));
        double error1 = std::abs(verification_d1 - d1);
        double error2 = std::abs(verification_d2 - d2);
        
        if (error1 > 0.5 || error2 > 0.5) { // 50cm tolerance
            ROS_WARN("Large localization errors: marker1 error=%.3f m, marker2 error=%.3f m", error1, error2);
            // Continue anyway but log warning
        }

        // Update poses
        baseline_pose_.x = xr;
        baseline_pose_.y = yr;
        baseline_pose_.theta = theta;
        current_pose_ = baseline_pose_;
        last_absolute_pose_time_ = ros::Time::now();

        // Update adjustment parameters for odometry integration
        initial_robot_x = xr;
        initial_robot_y = yr;
        initial_robot_theta = theta;
        adjustment_x_ = initial_robot_x - odom_x_;
        adjustment_y_ = initial_robot_y - odom_y_;
        adjustment_theta_ = initial_robot_theta - odom_theta_;

        ROS_INFO("Robot Pose (2-Landmark Depth): x=%.3f, y=%.3f, theta=%.3f degrees", 
                xr, yr, theta * 180.0 / M_PI);
        
        cv::imshow("ArUco Markers", output_image);
        cv::waitKey(1);

        return true;
    }


    double computeAngle(const std::pair<double, double>& p1, const std::pair<double, double>& p2) {
        // Convert image coordinates to angles using camera intrinsics
        double dx1 = (p1.first - cx_) / fx_;
        double dy1 = (p1.second - cy_) / fy_;
        double dx2 = (p2.first - cx_) / fx_;
        double dy2 = (p2.second - cy_) / fy_;
        double angle = std::acos((dx1 * dx2 + dy1 * dy2) / (std::sqrt(dx1 * dx1 + dy1 * dy1) * std::sqrt(dx2 * dx2 + dy2 * dy2)));
        return angle * 180.0 / M_PI; // Convert to degrees
    }

    double computeYaw(const std::pair<double, double>& marker_center, double marker_x, double marker_y, double robot_x, double robot_y) {
        // Compute direction to marker in world frame
        double dx = marker_x - robot_x;
        double dy = marker_y - robot_y;
        double world_angle = std::atan2(dy, dx);
        double image_angle = (marker_center.first - cx_) / fx_;
        double yaw = world_angle - (use_head_yaw_ ? head_yaw_ : 0.0) - image_angle; // Radians
        if (verbose_) {
            ROS_INFO("Head yaw: %.3f radians", head_yaw_);
        }
        return yaw;
    }

    int circle_circle_intersection(double x0, double y0, double r0,
        double x1, double y1, double r1,
        double *xi, double *yi,
        double *xi_prime, double *yi_prime)
    {
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;

    /* dx and dy are the vertical and horizontal distances between
    * the circle centers.
    */
    dx = x1 - x0;
    dy = y1 - y0;

    /* Determine the straight-line distance between the centers. */
    //d = sqrt((dy*dy) + (dx*dx));
    d = hypot(dx,dy); // Suggested by Keith Briggs

    /* Check for solvability. */
    if (d > (r0 + r1))
    {
    /* no solution. circles do not intersect. */
    return 0;
    }
    if (d < fabs(r0 - r1))
    {
    /* no solution. one circle is contained in the other */
    return 0;
    }

    /* 'point 2' is the point where the line through the circle
    * intersection points crosses the line between the circle
    * centers.  
    */

    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d);

    /* Determine the coordinates of point 2. */
    x2 = x0 + (dx * a/d);
    y2 = y0 + (dy * a/d);

    /* Determine the distance from point 2 to either of the
    * intersection points.
    */
    h = sqrt((r0*r0) - (a*a));

    /* Now determine the offsets of the intersection points from
    * point 2.
    */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    *xi = x2 + rx;
    *xi_prime = x2 - rx;
    *yi = y2 + ry;
    *yi_prime = y2 - ry;

    return 1;
    }

    int circle_centre(double x1, double y1, double x2, double y2, double alpha, double *xc1, double *yc1, double *xc2, double *yc2, double *r) {

        bool debug = false;
     
        double d;
        double h;
        double theta;
        double beta;
        double delta_x;
        double delta_y;
        double alphar;
        double temp_x;
        double temp_y;
     
        alphar =  3.14159 * (alpha / 180.0); // convert to radians
     
        d = sqrt((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
     
        if (alpha == 0 || alpha == 180) {
           h = 0;
           *r = 0;
        }
        else {
           h  = (d / 2) / tan(alphar);
           *r = (d / 2) / sin(alphar);
        }
     
        theta = atan2(y2-y1, x2-x1);
        beta  = theta - (3.14159 / 2);
        delta_x = h * cos(beta);
        delta_y = h * sin(beta);
     
        *xc1 = (x1 + x2)/2 - delta_x;  
        *yc1 = (y1 + y2)/2 - delta_y;  
     
        /* note: there is a second circle that satisfies the required condition */
        /* it is a reflection of the first circle in the given chord            */ 
        /* its centre is obtained by adding the delta_x and delta_y             */
     
        *xc2 = (x1 + x2)/2 + delta_x; 
        *yc2 = (y1 + y2)/2 + delta_y; 
         
        /* sort them in order of increasing distance from the origin */
     
        if ((*xc1 * *xc1 + *yc1 * *yc1) > (*xc2 * *xc2 + *yc2 * *yc2)) {
           temp_x = *xc1;
           *xc1 = *xc2;
           *xc2 = temp_x;
     
           temp_y = *yc1;
           *yc1 = *yc2;
           *yc2 = temp_y;
        }
        return 1;
    }

    void publishPose() {
        pose_pub_.publish(current_pose_);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robotLocalization");
    RobotLocalizationNode node;
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}