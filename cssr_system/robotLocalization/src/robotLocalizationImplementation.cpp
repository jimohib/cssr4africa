#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
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

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Transform odometry pose to map frame
        geometry_msgs::PoseStamped odom_pose, map_pose;
        odom_pose.header = msg->header;
        odom_pose.pose = msg->pose.pose;
        try {
            tf_buffer_.transform(odom_pose, map_pose, map_frame_, ros::Duration(0.1));
        } catch (const tf2::TransformException& ex) {
            ROS_WARN("Failed to transform odometry pose: %s", ex.what());
            return;
        }

        // Extract x, y, theta from transformed pose
        double x = map_pose.pose.position.x;
        double y = map_pose.pose.position.y;
        tf2::Quaternion q(
            map_pose.pose.orientation.x,
            map_pose.pose.orientation.y,
            map_pose.pose.orientation.z,
            map_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Compute deltas relative to last odometry pose
        double delta_x = x - last_odom_pose_.x;
        double delta_y = y - last_odom_pose_.y;
        double delta_theta = yaw - last_odom_pose_.theta;
        while (delta_theta > M_PI) delta_theta -= 2 * M_PI;
        while (delta_theta < -M_PI) delta_theta += 2 * M_PI;

        // Update last odometry pose
        last_odom_pose_.x = x;
        last_odom_pose_.y = y;
        last_odom_pose_.theta = yaw;

        // Apply deltas to baseline pose
        if (last_absolute_pose_time_.isValid() && (ros::Time::now() - last_absolute_pose_time_).toSec() < absolute_pose_timeout_) {
            // Rotate delta_x, delta_y by baseline theta
            double cos_theta = std::cos(baseline_pose_.theta);
            double sin_theta = std::sin(baseline_pose_.theta);
            current_pose_.x = baseline_pose_.x + delta_x * cos_theta + delta_y * sin_theta;
            current_pose_.y = baseline_pose_.y - delta_x * sin_theta + delta_y * cos_theta;
            current_pose_.theta = baseline_pose_.theta + delta_theta;
            while (current_pose_.theta > M_PI) current_pose_.theta -= 2 * M_PI;
            while (current_pose_.theta < -M_PI) current_pose_.theta += 2 * M_PI;
        } else {
            // Use raw odometry if no recent absolute pose
            current_pose_.x = x;
            current_pose_.y = y;
            current_pose_.theta = yaw;
        }

        publishPose();
        if (verbose_) {
            ROS_INFO("Odometry update: x=%.3f, y=%.3f, theta=%.3f degrees, deltas: dx=%.3f, dy=%.3f, dtheta=%.3f degrees",
                     current_pose_.x, current_pose_.y, current_pose_.theta * 180.0 / M_PI,
                     delta_x, delta_y, delta_theta * 180.0 / M_PI);
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

    bool setPoseCallback(cssr_system::SetPose::Request& req, cssr_system::SetPose::Response& res) {
        baseline_pose_.x = req.x;
        baseline_pose_.y = req.y;
        baseline_pose_.theta = req.theta * M_PI / 180.0; // Convert degrees to radians
        while (baseline_pose_.theta > M_PI) baseline_pose_.theta -= 2 * M_PI;
        while (baseline_pose_.theta < -M_PI) baseline_pose_.theta += 2 * M_PI;
        current_pose_ = baseline_pose_;
        last_absolute_pose_time_ = ros::Time::now();
        publishPose();
        if (verbose_) {
            ROS_INFO("Manually set pose: x=%.3f, y=%.3f, theta=%.3f degrees", current_pose_.x, current_pose_.y, req.theta);
        }
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


            ROS_INFO("Robot Pose: x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
            cv::imshow("ArUco Markers", output_image);
            cv::waitKey(1);

            publishPose();
            return true;
        }
    }

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
    
        if (marker_ids.size() < 3) {
            ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
            return false;
        }
    
        // Get distances from depth image
        std::vector<std::tuple<int, double, double, double>> markers; // id, x, y, distance
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            auto& corners = marker_corners[i];
            double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
            double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
            float distance = latest_depth_.at<float>(int(cy), int(cx)) / 1000.0; // Convert mm to m
            if (projected_landmarks_.find(marker_ids[i]) != projected_landmarks_.end() && !std::isnan(distance)) {
                markers.push_back({marker_ids[i], projected_landmarks_[marker_ids[i]].first, projected_landmarks_[marker_ids[i]].second, distance});
            }
            // ROS_INFO("Marker ID %d: Distance = %.3f m", marker_ids[i], distance);
        }

        std::sort(markers.begin(), markers.end(), 
            [](const std::tuple<int, double, double, double>& a, 
            const std::tuple<int, double, double, double>& b) {
                return std::get<3>(a) < std::get<3>(b); // Sort by distance (closest first)
            });

        ROS_INFO("Using %zu markers:", markers.size());
        for (const auto& marker : markers) {
            ROS_INFO("  Marker ID %d: Position (%.3f, %.3f), Distance = %.3f m", 
                    std::get<0>(marker), std::get<1>(marker), std::get<2>(marker), std::get<3>(marker));
        }
    
        if (markers.size() < 3) {
            ROS_WARN("Insufficient valid depth measurements");
            return false;
        }
    
        // Trilateration: Solve for (xr, yr) using three markers
        double x1 = std::get<1>(markers[0]), y1 = std::get<2>(markers[0]), d1 = std::get<3>(markers[0]);
        double x2 = std::get<1>(markers[1]), y2 = std::get<2>(markers[1]), d2 = std::get<3>(markers[1]);
        double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);
    
        // Simplified trilateration (intersection of two circles)
        double x12 = x2 - x1, y12 = y2 - y1;
        double d12 = std::sqrt(x12 * x12 + y12 * y12);
        double a = (d1 * d1 - d2 * d2 + d12 * d12) / (2.0 * d12);
        double h = std::sqrt(d1 * d1 - a * a);
        double xm = x1 + a * x12 / d12;
        double ym = y1 + a * y12 / d12;
    
        double xr1 = xm + h * (y2 - y1) / d12;
        double yr1 = ym - h * (x2 - x1) / d12;
        double xr2 = xm - h * (y2 - y1) / d12;
        double yr2 = ym + h * (x2 - x1) / d12;
    
        // Check which solution satisfies the third circle
        double dist1 = std::sqrt((xr1 - x3) * (xr1 - x3) + (yr1 - y3) * (yr1 - y3));
        double dist2 = std::sqrt((xr2 - x3) * (xr2 - x3) + (yr2 - y3) * (yr2 - y3));
        double xr, yr;
        if (std::abs(dist1 - d3) < std::abs(dist2 - d3)) {
            xr = xr1;
            yr = yr1;
        } else {
            xr = xr2;
            yr = yr2;
        }
    
        // Compute yaw
        double theta = computeYaw({marker_corners[0][0].x, marker_corners[0][0].y}, x1, y1, xr, yr);
    
        // Update pose
        baseline_pose_.x = xr;
        baseline_pose_.y = yr;
        baseline_pose_.theta = theta; // Radians
        current_pose_ = baseline_pose_;
        last_absolute_pose_time_ = ros::Time::now();
    
        // // Publish image
        // cv::Mat output_image = latest_image_.clone();
        // cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
        // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
        // image_pub_.publish(img_msg);
    

        ROS_INFO("Robot Pose (Depth): x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
        cv::imshow("ArUco Markers", output_image);
        cv::waitKey(1);

    
        publishPose();
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
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

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