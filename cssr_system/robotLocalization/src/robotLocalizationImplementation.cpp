/* robotLocalizationImplementation.cpp    Function definitions and implementation
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

#include "robotLocalization/robotLocalizationInterface.h"


void RobotLocalizationNode::initializePoseAdjustments() {
    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = angles::normalize_angle(initial_robot_theta - odom_theta_);
}

void RobotLocalizationNode::loadTopicNames() {
    try {
        std::ifstream file(topics_file_);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open topics file %s", topics_file_.c_str());
            return;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || line[0] == ';') {
                continue;
            }
            
            size_t pos = line.find_first_of(" \t");
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos);
                
                key.erase(key.find_last_not_of(" \t") + 1);
                
                value.erase(0, value.find_first_not_of(" \t"));
                
                value.erase(value.find_last_not_of(" \t") + 1);
                
                if (!key.empty() && !value.empty()) {
                    topic_map_[key] = value;
                }
            }
        }

        if (verbose_) {
            ROS_INFO("Loaded %zu topics from %s", topic_map_.size(), topics_file_.c_str());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error reading topics file %s: %s", topics_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::loadLandmarks() {
    try {
        // Read JSON file
        std::ifstream config_stream(landmark_file_);
        if (!config_stream.is_open()) {
            ROS_ERROR("Failed to open landmarks file %s", landmark_file_.c_str());
            return;
        }
        
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(config_stream, root)) {
            ROS_ERROR("Failed to parse landmarks file %s: %s", 
                     landmark_file_.c_str(), reader.getFormattedErrorMessages().c_str());
            return;
        }
        
        if (!root.isMember("landmarks") || !root["landmarks"].isArray()) {
            ROS_ERROR("No 'landmarks' array found in %s", landmark_file_.c_str());
            return;
        }
        
        const Json::Value& landmarks = root["landmarks"];
        for (const Json::Value& marker : landmarks) {
            // Check for required fields
            if (!marker.isMember("id") || !marker.isMember("x") || 
                !marker.isMember("y") || !marker.isMember("z")) {
                ROS_WARN("Skipping invalid landmark entry in %s", landmark_file_.c_str());
                continue;
            }
            
            Landmark3D lm;
            lm.id = marker["id"].asInt();
            lm.x = marker["x"].asDouble();
            lm.y = marker["y"].asDouble();
            lm.z = marker["z"].asDouble();

            landmarks3D_[lm.id] = lm;

            // Project to camera plane
            double dz = lm.z - camera_height_;
            double distance = std::sqrt(lm.x * lm.x + lm.y * lm.y + dz * dz);
            if (distance < 0.1) {
                ROS_WARN("Landmark %d too close to camera, skipping projection", lm.id);
                continue;
            }
            double scale = std::sqrt(lm.x * lm.x + lm.y * lm.y) / distance;
            double x_proj = lm.x * scale;
            double y_proj = lm.y * scale;

            projected_landmarks_[lm.id] = {x_proj, y_proj};

            if (verbose_) {
                ROS_INFO("Loaded landmark ID %d: (%.2f, %.2f, %.2f) -> Projected (%.2f, %.2f)",
                         lm.id, lm.x, lm.y, lm.z, x_proj, y_proj);
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error loading landmarks from %s: %s", landmark_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::loadCameraInfoFromFile() {
    try {
        // Read JSON file
        std::ifstream config_stream(camera_info_file_);
        if (!config_stream.is_open()) {
            ROS_ERROR("Failed to open camera info file %s", camera_info_file_.c_str());
            return;
        }
        
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(config_stream, root)) {
            ROS_ERROR("Failed to parse camera info file %s: %s", 
                     camera_info_file_.c_str(), reader.getFormattedErrorMessages().c_str());
            return;
        }

        if (!root.isMember("cameraInfo")) {
            ROS_ERROR("No 'cameraInfo' object found in %s", camera_info_file_.c_str());
            return;
        }

        const Json::Value& camera_info = root["cameraInfo"];
        
        // Check for required fields
        if (!camera_info.isMember("fx") || !camera_info.isMember("fy") || 
            !camera_info.isMember("cx") || !camera_info.isMember("cy")) {
            ROS_ERROR("Missing required camera intrinsic parameters in %s", camera_info_file_.c_str());
            return;
        }
        
        fx_ = camera_info["fx"].asDouble();
        fy_ = camera_info["fy"].asDouble();
        cx_ = camera_info["cx"].asDouble();
        cy_ = camera_info["cy"].asDouble();
        
        if (fx_ <= 0 || fy_ <= 0 || cx_ <= 0 || cy_ <= 0) {
            ROS_ERROR("Invalid camera intrinsics in %s", camera_info_file_.c_str());
            return;
        }
        
        camera_info_received_ = true;
        ROS_INFO("Loaded camera intrinsics from file: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                 fx_, fy_, cx_, cy_);
                 
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load camera info from %s: %s", camera_info_file_.c_str(), e.what());
    }
}

void RobotLocalizationNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
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
    ROS_INFO("Received camera intrinsics from camera: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
}

void RobotLocalizationNode::cameraInfoTimeoutCallback(const ros::TimerEvent& event) {
    if (!camera_info_received_) {
        ROS_WARN("No camera info received within %.1f seconds, falling back to %s", camera_info_timeout_, camera_info_file_.c_str());
        loadCameraInfoFromFile();
    }
}

void RobotLocalizationNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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
    // pose_msg.theta = angles::to_degrees(relative_pose.theta);
    pose_msg.theta = fmod(angles::to_degrees(relative_pose.theta) + 360.0, 360.0);


    pose_pub_.publish(pose_msg);

    if (verbose_) {
        ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f degrees", current_x, current_y, angles::to_degrees(current_theta));
    }
}

void RobotLocalizationNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Optional using IMU angular velocity
}

void RobotLocalizationNode::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    bool found = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == head_yaw_joint_name_) {
            head_yaw_ = msg->position[i];
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

void RobotLocalizationNode::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (verbose_) {
            ROS_INFO("Received image: width=%d, height=%d", latest_image_.cols, latest_image_.rows);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void RobotLocalizationNode::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        latest_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image;
        if (verbose_) {
            ROS_INFO("Received depth image: width=%d, height=%d", latest_depth_.cols, latest_depth_.rows);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

bool RobotLocalizationNode::setPoseCallback(cssr_system::setPose::Request& req, cssr_system::setPose::Response& res) {
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

bool RobotLocalizationNode::resetPoseCallback(cssr_system::resetPose::Request& req, cssr_system::resetPose::Response& res) {
    if (computeAbsolutePoseWithActiveScanning()) {
        res.success = true;
        ROS_INFO("Pose reset successfully");
    } else {
        res.success = false;
        ROS_WARN("Failed to reset pose");
    }
    return true;
}

void RobotLocalizationNode::resetTimerCallback(const ros::TimerEvent& event) {
    computeAbsolutePoseWithActiveScanning();
}

// bool RobotLocalizationNode::triangulateRobotPosition(double x1, double y1, double x2, double y2, double x3, double y3, double alpha1, double alpha2, double &xr, double &yr) {
    
//     double xc1a, yc1a, xc1b, yc1b, xc2a, yc2a, xc2b, yc2b, r1, r2;
//     circle_centre(x2, y2, x1, y1, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
//     circle_centre(x3, y3, x2, y2, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);

//     if (verbose_) {
//         ROS_INFO("Circle 1 centers: (%.3f, %.3f), (%.3f, %.3f) r=%.3f", xc1a, yc1a, xc1b, yc1b, r1);
//         ROS_INFO("Circle 2 centers: (%.3f, %.3f), (%.3f, %.3f) r=%.3f", xc2a, yc2a, xc2b, yc2b, r2);
//     }

//     struct CirclePair {
//         double xc1, yc1, xc2, yc2;
//         std::string name;
//     };

//     // Try all 4 combinations of circle centres
//     std::vector<CirclePair> combos = {
//         {xc1a, yc1a, xc2a, yc2a, "closest-closest"},
//         {xc1a, yc1a, xc2b, yc2b, "closest-farthest"},
//         {xc1b, yc1b, xc2a, yc2a, "farthest-closest"},
//         {xc1b, yc1b, xc2b, yc2b, "farthest-farthest"}
//     };

//     double best_x = 0, best_y = 0;
//     double best_score = -1;
//     bool found_valid = false;

//     if (verbose_) ROS_INFO("=== Testing all circle combinations ===");

//     for (auto &combo : combos) {

//         double xi1, yi1, xi2, yi2;
//         int result = circle_circle_intersection(
//             combo.xc1, combo.yc1, r1,
//             combo.xc2, combo.yc2, r2,
//             &xi1, &yi1, &xi2, &yi2);

//         if (result == 0) {
//             ROS_INFO("%s: No circle intersection", combo.name.c_str());
//             continue;
//         }
        
//         // Check both intersection points
//         std::vector<std::pair<double, double>> candidates = {
//             {xi1, yi1}, {xi2, yi2}
//         };

//         for (size_t i = 0; i < candidates.size(); i++) {

//             double xr_test = candidates[i].first;
//             double yr_test = candidates[i].second;

//             // Calculate distances to landmarks
//             double dist1 = std::hypot(xr_test - x1, yr_test - y1);
//             double dist2 = std::hypot(xr_test - x2, yr_test - y2);
//             double dist3 = std::hypot(xr_test - x3, yr_test - y3);

//             double min_dist = std::min({dist1, dist2, dist3});
//             double avg_dist = (dist1 + dist2 + dist3) / 3.0;

//             // Reject obvious outliers (extremely close to landmarks)
//             if (min_dist < 0.01) continue;

//             // Calculate triangle areas formed by robot and landmark pairs
//             double area1 = std::abs((xr_test - x1)*(y2 - y1) - (x2 - x1)*(yr_test - y1)) / 2.0;
//             double area2 = std::abs((xr_test - x2)*(y3 - y2) - (x3 - x2)*(yr_test - y2)) / 2.0;
//             double area3 = std::abs((xr_test - x1)*(y3 - y1) - (x3 - x1)*(yr_test - y1)) / 2.0;
//             double avg_area = (area1 + area2 + area3) / 3.0;

//             // Calculate viewing angles from robot to landmark pairs
//             double a12 = std::abs(std::atan2(y1 - yr_test, x1 - xr_test) - std::atan2(y2 - yr_test, x2 - xr_test));
//             double a23 = std::abs(std::atan2(y2 - yr_test, x2 - xr_test) - std::atan2(y3 - yr_test, x3 - xr_test));
//             double a13 = std::abs(std::atan2(y1 - yr_test, x1 - xr_test) - std::atan2(y3 - yr_test, x3 - xr_test));

//             // Normalize angles to [0, π]
//             if (a12 > M_PI) a12 = 2*M_PI - a12;
//             if (a23 > M_PI) a23 = 2*M_PI - a23;
//             if (a13 > M_PI) a13 = 2*M_PI - a13;

//             double min_a = std::min({a12, a23, a13});
//             double max_a = std::max({a12, a23, a13});
//             double min_deg = min_a * 180.0 / M_PI;
//             double max_deg = max_a * 180.0 / M_PI;

//             // Area quality score (larger triangulation areas = better precision)
//             double area_score = avg_area;

//             // Distance reasonableness (prefer moderate distances, avoid extremes)
//             double dist_score = 1.0;
//             if (avg_dist < 1.0 || avg_dist > 15.0) dist_score = 0.1;        // Penalize unrealistic distances
//             else if (avg_dist < 2.0 || avg_dist > 10.0) dist_score = 0.5;   // Penalize marginal distances
//             else dist_score = 1.0 / (1.0 + std::abs(avg_dist - 5.0));       // Prefer approx. 5m

//             // Angle quality (avoid degenerate triangulation)
//             double angle_score = 1.0;
//             if (min_deg < 10.0) angle_score = 0.05;                          // Severe penalty for very acute angles
//             else if (min_deg < 20.0) angle_score = 0.3;                      // Moderate penalty for acute angles
//             else if (min_deg > 25.0 && max_deg < 100.0) angle_score = 2.0;   // Reward well-distributed angles
//             else angle_score = 1.0;

//             // Landmark separation score (avoid degenerate landmark configurations)
//             double sep = std::min({
//                 std::hypot(x2-x1, y2-y1),
//                 std::hypot(x3-x2, y3-y2),
//                 std::hypot(x3-x1, y3-y1)
//             });
//             double sep_score = (sep > 1.0) ? 1.0 : 0.1;

//             // Proximity penalty (slightly penalize being too close to any landmark)
//             double prox_pen = 1.0;
//             if (min_dist < 0.5) prox_pen = 0.2;
//             else if (min_dist < 1.0) prox_pen = 0.7;

//             // Combine all geometric factors
//             double score = area_score * dist_score * angle_score * sep_score * prox_pen;

//             if (score > best_score) {
//                 best_score = score;
//                 best_x = xr_test;
//                 best_y = yr_test;
//                 found_valid = true;
//             }
//         }
//     }

//     if (!found_valid) return false;

//     xr = best_x;
//     yr = best_y;
//     return true;
// }

bool RobotLocalizationNode::triangulateRobotPosition(double x1, double y1, double x2, double y2, double x3, double y3, double alpha1, double alpha2, double &xr, double &yr) {
    
    // Precompute landmark distances once
    double d12 = std::hypot(x2-x1, y2-y1);
    double d23 = std::hypot(x3-x2, y3-y2);
    double d13 = std::hypot(x3-x1, y3-y1);

    // Compute circle centers
    double xc1a, yc1a, xc1b, yc1b, xc2a, yc2a, xc2b, yc2b, r1, r2;
    circle_centre(x2, y2, x1, y1, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
    circle_centre(x3, y3, x2, y2, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);

    struct C { double xc1, yc1, xc2, yc2; };

    // Try all 4 combinations of circle centres
    std::vector<C> combos = {
        {xc1a, yc1a, xc2a, yc2a},
        {xc1a, yc1a, xc2b, yc2b},
        {xc1b, yc1b, xc2a, yc2a},
        {xc1b, yc1b, xc2b, yc2b}
    };

    double best_score = -1;
    double best_x = 0, best_y = 0;
    bool found = false;

    for (auto &c : combos) {
        double xi1, yi1, xi2, yi2;
        if (!circle_circle_intersection(c.xc1, c.yc1, r1, c.xc2, c.yc2, r2, &xi1, &yi1, &xi2, &yi2))
            continue;

        // Check both intersection points
        std::vector<std::pair<double,double>> candidates = {{ xi1, yi1 }, { xi2, yi2 }};

        for (const auto &p : candidates) {
            double px = p.first;
            double py = p.second;

            // Calculate distances to landmarks
            double d1 = std::hypot(px - x1, py - y1);
            double d2 = std::hypot(px - x2, py - y2);
            double d3 = std::hypot(px - x3, py - y3);

            // Reject obvious outliers (extremely close to landmarks)
            double min_d = std::min({d1, d2, d3});
            if (min_d < 0.01) continue;

            double avg_d = (d1 + d2 + d3) / 3.0;

            // Calculate triangle areas formed by robot and landmark pairs
            double area1 = std::abs((px-x1)*(y2-y1)-(x2-x1)*(py-y1))/2.0;
            double area2 = std::abs((px-x2)*(y3-y2)-(x3-x2)*(py-y2))/2.0;
            double area3 = std::abs((px-x1)*(y3-y1)-(x3-x1)*(py-y1))/2.0;
            double avg_area = (area1 + area2 + area3) / 3.0;

            auto norm = [](double a){
                return (a > M_PI) ? 2*M_PI - a : a;
            };

            // Calculate viewing angles from robot to landmark pairs
            double a12 = norm(std::abs(std::atan2(y1-py, x1-px) - std::atan2(y2-py, x2-px)));
            double a23 = norm(std::abs(std::atan2(y2-py, x2-px) - std::atan2(y3-py, x3-px)));
            double a13 = norm(std::abs(std::atan2(y1-py, x1-px) - std::atan2(y3-py, x3-px)));

            double min_a = std::min({a12, a23, a13});
            double max_a = std::max({a12, a23, a13});
            double min_deg = min_a * 180.0/M_PI;
            double max_deg = max_a * 180.0/M_PI;

            // Area quality score (larger triangulation areas = better precision)
            double area_score = avg_area;

            // Distance reasonableness (prefer moderate distances, avoid extremes)
            double dist_score =
                (avg_d < 1.0 || avg_d > 15.0) ? 0.1 :
                (avg_d < 2.0 || avg_d > 10.0) ? 0.5 :
                1.0 / (1.0 + std::abs(avg_d - 5.0));

            // Angle quality (avoid degenerate triangulation)
            double angle_score =
                (min_deg < 10.0) ? 0.05 :
                (min_deg < 20.0) ? 0.3 :
                (min_deg > 25.0 && max_deg < 100.0) ? 2.0 :
                1.0;

            // Landmark separation score (avoid degenerate landmark configurations)
            double sep = std::min({d12, d23, d13});
            double sep_score = (sep > 1.0) ? 1.0 : 0.1;

            // Proximity penalty (slightly penalize being too close to any landmark)
            double prox =
                (min_d < 0.5) ? 0.2 :
                (min_d < 1.0) ? 0.7 :
                1.0;

            // Combine all geometric factors
            double score = area_score * dist_score * angle_score * sep_score * prox;

            if (score > best_score) {
                best_score = score;
                best_x = px;
                best_y = py;
                found = true;
            }
        }
    }

    if (!found) return false;

    xr = best_x;
    yr = best_y;
    return true;
}

double RobotLocalizationNode::computeAngle(const std::pair<double, double>& center1, const std::pair<double, double>& center2) {
    // Output of marker center
    if (verbose_){
        ROS_INFO("Point 1: (%.2f, %.2f), Point 2: (%.2f, %.2f)", center1.first, center1.second, center2.first, center2.second);
    }
    
    // Convert pixel coordinates to normalized camera coordinates
    double x1_norm = (center1.first - cx_) / fx_;
    double y1_norm = (center1.second - cy_) / fy_;
    double x2_norm = (center2.first - cx_) / fx_;
    double y2_norm = (center2.second - cy_) / fy_;
    
    // Create 3D direction vectors
    cv::Vec3d dir1(x1_norm, y1_norm, 1.0);
    cv::Vec3d dir2(x2_norm, y2_norm, 1.0);
    
    // Normalize the vectors
    dir1 = dir1 / cv::norm(dir1);
    dir2 = dir2 / cv::norm(dir2);
    
    // Compute angle using dot product
    double dot_product = dir1.dot(dir2);
    // Clamp manually to avoid numerical errors in acos
    if (dot_product > 1.0) dot_product = 1.0;
    if (dot_product < -1.0) dot_product = -1.0;
    
    double angle_rad = std::acos(dot_product);
    
    return angle_rad * 180.0 / M_PI; // Convert to degrees
}

double RobotLocalizationNode::computeYaw(const std::pair<double, double>& marker_center, double marker_x, double marker_y, double robot_x, double robot_y) {
    // Compute direction to marker in world frame
    double dx = marker_x - robot_x;
    double dy = marker_y - robot_y;
    double world_angle = std::atan2(dy, dx);
    
    // Convert pixel offset to angle in camera frame (horizontal only)
    double pixel_offset_x = marker_center.first - cx_;
    double image_angle = std::atan(pixel_offset_x / fx_);

    double raw_yaw = world_angle + image_angle - (use_head_yaw_ ? head_yaw_ : 0.0);
    
    // Normalize raw_yaw first
    while (raw_yaw > M_PI) raw_yaw -= 2 * M_PI;
    while (raw_yaw < -M_PI) raw_yaw += 2 * M_PI;
    
    // Calibrate yaw
    double calibrated_yaw = raw_yaw + (346.0 * M_PI / 180.0);
    
    // Final normalization
    while (calibrated_yaw > M_PI) calibrated_yaw -= 2 * M_PI;
    while (calibrated_yaw < -M_PI) calibrated_yaw += 2 * M_PI;

    double yaw_positive = calibrated_yaw * 180.0 / M_PI;
    if (yaw_positive < 0) {
        yaw_positive += 360.0;
    }

    if (verbose_) {
        ROS_INFO("Theta=%.3f degrees (positive: %.3f degrees)", calibrated_yaw * 180.0 / M_PI, yaw_positive);
    }

    return calibrated_yaw;
}

int RobotLocalizationNode::circle_centre(double x1, double y1, double x2, double y2, double alpha, double *xc1, double *yc1, double *xc2, double *yc2, double *r) {
    bool debug = false;
    double d, h, theta, beta, delta_x, delta_y, alphar, temp_x, temp_y;
    
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

int RobotLocalizationNode::circle_circle_intersection(double x0, double y0, double r0, double x1, double y1, double r1, double *xi, double *yi, double *xi_prime, double *yi_prime) {
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

// Move head to a specific yaw and pitch position (for head scanning)
bool RobotLocalizationNode::moveHeadToPosition(double yaw, double pitch) {
    if (!head_control_client_ || !head_control_client_->isServerConnected()) {
        ROS_WARN("Head control client not connected");
        return false;
    }

    // Clamp yaw and pitch to safe limits
    yaw = std::max(-2.085, std::min(2.085, yaw));      // -119.4° to 119.4°
    pitch = std::max(-0.706, std::min(0.445, pitch));  // -40.4° to 25.5°

    // Create trajectory goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"HeadPitch", "HeadYaw"};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = {pitch, yaw};
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    // Send goal and wait
    head_control_client_->sendGoal(goal);
    bool success = head_control_client_->waitForResult(ros::Duration(3.0));

    if (!success) {
        ROS_WARN("Head movement timed out");
        return false;
    }

    // Wait for image to stabilize
    ros::Duration(0.5).sleep();

    if (verbose_) {
        ROS_INFO("Moved head to yaw=%.2f, pitch=%.2f", yaw, pitch);
    }

    return true;
}

// Detect markers in current view and store them in memory
void RobotLocalizationNode::detectAndStoreMarkers(double current_head_yaw) {
    if (latest_image_.empty()) {
        ROS_WARN("No image available for marker detection");
        return;
    }

    // Detect ArUco markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    cv::aruco::detectMarkers(latest_image_, dictionary, marker_corners, marker_ids);

    if (verbose_ && !marker_ids.empty()) {
        ROS_INFO("Detected %zu markers at head yaw %.2f", marker_ids.size(), current_head_yaw);
    }

    // Store each detected marker
    for (size_t i = 0; i < marker_ids.size(); ++i) {
        int id = marker_ids[i];

        // Check if this marker is already in memory (update if so)
        bool found = false;
        for (auto& stored : marker_memory_) {
            if (stored.id == id) {
                // Update existing marker
                stored.corners = marker_corners[i];
                stored.head_yaw = current_head_yaw;
                stored.timestamp = ros::Time::now();

                // Update center
                double cx = (marker_corners[i][0].x + marker_corners[i][1].x +
                            marker_corners[i][2].x + marker_corners[i][3].x) / 4.0;
                double cy = (marker_corners[i][0].y + marker_corners[i][1].y +
                            marker_corners[i][2].y + marker_corners[i][3].y) / 4.0;
                stored.center = {cx, cy};

                // Update body-frame direction
                double x_norm = (cx - cx_) / fx_;
                double y_norm = (cy - cy_) / fy_;
                cv::Vec3d dir_cam(x_norm, y_norm, 1.0);
                dir_cam = dir_cam / cv::norm(dir_cam);

                double cos_yaw = std::cos(current_head_yaw);
                double sin_yaw = std::sin(current_head_yaw);
                stored.direction_body = cv::Vec3d(
                    dir_cam[2] * sin_yaw + dir_cam[0] * cos_yaw,
                    dir_cam[1],
                    dir_cam[2] * cos_yaw - dir_cam[0] * sin_yaw
                );
                stored.direction_body = stored.direction_body / cv::norm(stored.direction_body);

                found = true;
                break;
            }
        }

        // Add new marker to memory
        if (!found) {
            DetectedMarker marker;
            marker.id = id;
            marker.corners = marker_corners[i];
            marker.head_yaw = current_head_yaw;
            marker.timestamp = ros::Time::now();

            // Compute center
            double cx = (marker_corners[i][0].x + marker_corners[i][1].x +
                        marker_corners[i][2].x + marker_corners[i][3].x) / 4.0;
            double cy = (marker_corners[i][0].y + marker_corners[i][1].y +
                        marker_corners[i][2].y + marker_corners[i][3].y) / 4.0;
            marker.center = {cx, cy};

            // Convert to normalized camera coordinates
            double x_norm = (cx - cx_) / fx_;
            double y_norm = (cy - cy_) / fy_;

            // Create 3D direction vector in camera frame
            cv::Vec3d dir_cam(x_norm, y_norm, 1.0);
            dir_cam = dir_cam / cv::norm(dir_cam);

            // Transform to robot body frame
            // Simple 2D rotation in horizontal plane (camera X-Z plane)
            // When head turns left (positive yaw), camera rotates left
            // Camera Z (forward) becomes body direction at angle head_yaw
            double cos_yaw = std::cos(current_head_yaw);
            double sin_yaw = std::sin(current_head_yaw);

            // Rotation: camera forward (Z) and camera right (X) to body frame
            marker.direction_body = cv::Vec3d(
                dir_cam[2] * sin_yaw + dir_cam[0] * cos_yaw,  // body X (forward)
                dir_cam[1],                                     // body Y (left/vertical)
                dir_cam[2] * cos_yaw - dir_cam[0] * sin_yaw   // body Z (up)
            );
            marker.direction_body = marker.direction_body / cv::norm(marker.direction_body);

            marker_memory_.push_back(marker);

            if (verbose_) {
                ROS_INFO("Stored new marker ID %d at head yaw %.2f rad, body direction: [%.3f, %.3f, %.3f]",
                         id, current_head_yaw,
                         marker.direction_body[0], marker.direction_body[1], marker.direction_body[2]);
            }
        }
    }
}

// Remove old markers from memory based on timeout
void RobotLocalizationNode::cleanupOldMarkers() {
    ros::Time now = ros::Time::now();
    marker_memory_.erase(
        std::remove_if(marker_memory_.begin(), marker_memory_.end(),
            [this, now](const DetectedMarker& m) {
                return (now - m.timestamp).toSec() > marker_memory_timeout_;
            }),
        marker_memory_.end()
    );
}

// Select best N markers from memory based on various criteria
std::vector<DetectedMarker> RobotLocalizationNode::selectBestMarkers(int count) {
    if (marker_memory_.size() <= count) {
        return marker_memory_;
    }

    // Score each marker based on:
    // 1. Recency (newer is better)
    // 2. Distance from center of image (closer to center is better)
    // 3. Known landmark (must be in projected_landmarks_)

    std::vector<std::pair<double, DetectedMarker>> scored_markers;
    ros::Time now = ros::Time::now();

    for (const auto& marker : marker_memory_) {
        // Skip unknown landmarks
        if (projected_landmarks_.find(marker.id) == projected_landmarks_.end()) {
            continue;
        }

        double score = 0.0;

        // Recency score (0-10 points, decays over time)
        double age = (now - marker.timestamp).toSec();
        score += 10.0 * std::exp(-age / marker_memory_timeout_);

        // Distance from center score (0-10 points)
        double dx = marker.center.first - cx_;
        double dy = marker.center.second - cy_;
        double dist_from_center = std::sqrt(dx*dx + dy*dy);
        double max_dist = std::sqrt(cx_*cx_ + cy_*cy_);
        score += 10.0 * (1.0 - dist_from_center / max_dist);

        scored_markers.push_back({score, marker});
    }

    // Sort by score (highest first)
    std::sort(scored_markers.begin(), scored_markers.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });

    // Return top N markers
    std::vector<DetectedMarker> best_markers;
    for (int i = 0; i < count && i < scored_markers.size(); ++i) {
        best_markers.push_back(scored_markers[i].second);
    }

    return best_markers;
}

// Compute angle between two markers accounting for head yaw difference
// double RobotLocalizationNode::computeAngleWithHeadYaw(const DetectedMarker& m1, const DetectedMarker& m2) {
//     // Get pixel coordinates of marker centers
//     double cx1 = m1.center.first;
//     double cy1 = m1.center.second;
//     double cx2 = m2.center.first;
//     double cy2 = m2.center.second;

//     // Convert pixel coordinates to normalized camera coordinates
//     double x1_norm = (cx1 - cx_) / fx_;
//     double y1_norm = (cy1 - cy_) / fy_;
//     double x2_norm = (cx2 - cx_) / fx_;
//     double y2_norm = (cy2 - cy_) / fy_;

//     // Create 3D direction vectors
//     cv::Vec3d dir1(x1_norm, y1_norm, 1.0);
//     cv::Vec3d dir2(x2_norm, y2_norm, 1.0);
    
//     // Normalize the vectors
//     dir1 = dir1 / cv::norm(dir1);
//     dir2 = dir2 / cv::norm(dir2);
    
//     // Compute angle using dot product
//     double dot_product = dir1.dot(dir2);
//     // Clamp manually to avoid numerical errors in acos
//     if (dot_product > 1.0) dot_product = 1.0;
//     if (dot_product < -1.0) dot_product = -1.0;
    
//     double angle_rad = std::acos(dot_product);
    
//     return angle_rad * 180.0 / M_PI; // Convert to degrees
// }

std::pair<double, double> RobotLocalizationNode::computeMarkerBearing(const DetectedMarker& marker) {
    // Compute horizontal angle in camera frame (angle from optical axis)
    double angle_cam = std::atan((marker.center.first - cx_) / fx_);
    
    // Transform to robot base frame by adding head yaw
    double angle_base = angle_cam + marker.head_yaw;
    
    // Return as unit direction vector in base frame
    return {std::cos(angle_base), std::sin(angle_base)};
}

double RobotLocalizationNode::computeAngleWithHeadYaw(const DetectedMarker& m1, const DetectedMarker& m2) {
    // Use pre-computed body-frame directions
    // These were already transformed when the markers were detected and stored
    const cv::Vec3d& dir1_body = m1.direction_body;
    const cv::Vec3d& dir2_body = m2.direction_body;

    // Compute angle using dot product
    double dot_product = dir1_body.dot(dir2_body);

    // Clamp to avoid numerical errors in acos
    if (dot_product > 1.0) dot_product = 1.0;
    if (dot_product < -1.0) dot_product = -1.0;

    double angle_rad = std::acos(dot_product);
    double angle_deg = angle_rad * 180.0 / M_PI;

    if (verbose_) {
        ROS_INFO("Angle between markers (using body-frame directions): %.2f degrees", angle_deg);
        ROS_INFO("  Marker 1 body dir: [%.3f, %.3f, %.3f] at head_yaw=%.2f",
                 dir1_body[0], dir1_body[1], dir1_body[2], m1.head_yaw);
        ROS_INFO("  Marker 2 body dir: [%.3f, %.3f, %.3f] at head_yaw=%.2f",
                 dir2_body[0], dir2_body[1], dir2_body[2], m2.head_yaw);
    }

    // Convert to degrees
    return angle_deg;
}

// Compute absolute pose with active head scanning
// bool RobotLocalizationNode::computeAbsolutePoseWithActiveScanning() {
//     if (!enable_active_scanning_) {
//         return computeAbsolutePose();
//     }

//     if (projected_landmarks_.empty()) {
//         ROS_WARN("No landmarks loaded");
//         return false;
//     }
//     if (!camera_info_received_) {
//         ROS_WARN("Camera intrinsics not received");
//         return false;
//     }

//     // Set scanning flag
//     is_scanning_ = true;
//     initial_head_yaw_ = head_yaw_;

//     // Clean up old markers
//     cleanupOldMarkers();

//     // Detect markers in current view
//     detectAndStoreMarkers(head_yaw_);

//     if (verbose_) {
//         ROS_INFO("Marker memory size after current view: %zu", marker_memory_.size());
//     }

//     // If we already have 3+ unique markers, proceed with localization
//     if (marker_memory_.size() >= 3) {
//         if (verbose_) {
//             ROS_INFO("Found %zu markers in current view. Proceeding with localization.", marker_memory_.size());
//         }
//     } else {
//         // Need to scan for more markers
//         int markers_needed = 3 - marker_memory_.size();
//         ROS_INFO("Only found %zu markers in current view. Scanning for %d more...",
//                  marker_memory_.size(), markers_needed);

//         // Scan through predefined positions
//         for (double scan_yaw : scan_positions_) {
//             // Skip if this is close to current position (already scanned)
//             if (std::abs(scan_yaw - initial_head_yaw_) < 0.1) {
//                 continue;
//             }

//             // Move head to scan position
//             if (!moveHeadToPosition(scan_yaw, 0.0)) {
//                 ROS_WARN("Failed to move head to scan position %.2f", scan_yaw);
//                 continue;
//             }

//             // Wait for new image (handled in moveHeadToPosition)
//             ros::spinOnce();

//             // Detect and store markers at this position
//             detectAndStoreMarkers(scan_yaw);

//             if (verbose_) {
//                 ROS_INFO("Marker memory size after scanning yaw=%.2f: %zu", scan_yaw, marker_memory_.size());
//             }

//             // Stop scanning if we have enough markers
//             if (marker_memory_.size() >= 3) {
//                 ROS_INFO("Found sufficient markers (%zu) after scanning", marker_memory_.size());
//                 break;
//             }
//         }

//         // Restore head to initial position
//         moveHeadToPosition(initial_head_yaw_, 0.0);
//     }

//     // Check if we have enough markers
//     if (marker_memory_.size() < 3) {
//         ROS_WARN("Active scanning complete. Only found %zu markers (need 3)", marker_memory_.size());
//         is_scanning_ = false;

//         // Publish marker image showing what we found
//         if (!latest_image_.empty()) {
//             cv::Mat output_image = latest_image_.clone();
//             std::vector<int> ids;
//             std::vector<std::vector<cv::Point2f>> corners;
//             for (const auto& m : marker_memory_) {
//                 ids.push_back(m.id);
//                 corners.push_back(m.corners);
//             }
//             if (!ids.empty()) {
//                 cv::aruco::drawDetectedMarkers(output_image, corners, ids);
//             }
//             sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
//             image_pub_.publish(img_msg);
//         }

//         return false;
//     }

//     // Select best 3 markers from memory
//     // std::vector<DetectedMarker> best_markers = selectBestMarkers(3);

//     // if (best_markers.size() < 3) {
//     //     ROS_WARN("Could not select 3 valid markers from memory");
//     //     is_scanning_ = false;
//     //     return false;
//     // }

//     // Filter to only known landmarks
//     std::vector<DetectedMarker> valid_markers;
//     for (const auto& m : marker_memory_) {
//         if (projected_landmarks_.find(m.id) != projected_landmarks_.end()) {
//             valid_markers.push_back(m);
//         }
//     }

//     if (valid_markers.size() < 3) {
//         ROS_WARN("Could not find 3 valid markers from memory");
//         is_scanning_ = false;
//         return false;
//     }

//     // Sort markers by their WORLD X-COORDINATE (left to right in world frame)
//     // This is robust and doesn't suffer from angle wrapping issues
//     std::sort(valid_markers.begin(), valid_markers.end(), 
//         [this](const DetectedMarker& a, const DetectedMarker& b) {
//             double x_a = projected_landmarks_[a.id].first;
//             double x_b = projected_landmarks_[b.id].first;
//             return x_a < x_b;
//         });

//     if (verbose_) {
//         ROS_INFO("Sorted markers by world x-coordinate:");
//         for (size_t i = 0; i < valid_markers.size() && i < 5; ++i) {
//             double x = projected_landmarks_[valid_markers[i].id].first;
//             double y = projected_landmarks_[valid_markers[i].id].second;
//             double bearing = std::atan((valid_markers[i].center.first - cx_) / fx_) + valid_markers[i].head_yaw;
//             ROS_INFO("  %zu: ID=%d at world (%.3f, %.3f), bearing=%.3f°, head_yaw=%.3f°", 
//                     i, valid_markers[i].id, x, y, bearing * 180.0/M_PI, 
//                     valid_markers[i].head_yaw * 180.0/M_PI);
//         }
//     }

//     // Take first 3 markers (they're now in world spatial order: left to right)
//     std::vector<DetectedMarker> best_markers;
//     for (int i = 0; i < 3 && i < valid_markers.size(); ++i) {
//         best_markers.push_back(valid_markers[i]);
//     }

//     if (best_markers.size() < 3) {
//         ROS_WARN("Could not select 3 markers");
//         is_scanning_ = false;
//         return false;
//     }

//     // Extract marker IDs and check if they are known landmarks
//     int id1 = best_markers[0].id;
//     int id2 = best_markers[1].id;
//     int id3 = best_markers[2].id;

//     if (projected_landmarks_.find(id1) == projected_landmarks_.end() ||
//         projected_landmarks_.find(id2) == projected_landmarks_.end() ||
//         projected_landmarks_.find(id3) == projected_landmarks_.end()) {
//         ROS_WARN("Unknown marker IDs detected: %d, %d, %d", id1, id2, id3);
//         is_scanning_ = false;
//         return false;
//     }

//     double x1 = projected_landmarks_[id1].first, y1 = projected_landmarks_[id1].second;
//     double x2 = projected_landmarks_[id2].first, y2 = projected_landmarks_[id2].second;
//     double x3 = projected_landmarks_[id3].first, y3 = projected_landmarks_[id3].second;

//     ROS_INFO("Using found ArUco markers:");
//     ROS_INFO("Marker 1: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id1, x1, y1, best_markers[0].head_yaw);
//     ROS_INFO("Marker 2: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id2, x2, y2, best_markers[1].head_yaw);
//     ROS_INFO("Marker 3: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id3, x3, y3, best_markers[2].head_yaw);

//     // Check for collinear markers and small landmark triangle area
//     double landmark_triangle_area = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0;
//     if (landmark_triangle_area < 0.5) {
//         ROS_WARN("Landmark triangle area too small (%.3f), rejecting configuration", landmark_triangle_area);
//         is_scanning_ = false;
//         return false;
//     }
//     double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
//     if (std::abs(cross_product) < 0.01) {
//         ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
//         is_scanning_ = false;
//         return false;
//     }

//     // Compute angles between markers accounting for head yaw
//     double alpha1 = computeAngleWithHeadYaw(best_markers[0], best_markers[1]);
//     double alpha2 = computeAngleWithHeadYaw(best_markers[1], best_markers[2]);

//     // Reject extreme angles
//     if (alpha1 < 5.0 || alpha2 < 5.0 || alpha1 > 120.0 || alpha2 > 120.0) {
//         ROS_WARN("Angles too extreme (alpha1=%.3f, alpha2=%.3f), rejecting configuration", alpha1, alpha2);
//         is_scanning_ = false;
//         return false;
//     }

//     if (verbose_) {
//         ROS_INFO("Computed angles: alpha1=%.3f, alpha2=%.3f", alpha1, alpha2);
//     }

//     ROS_INFO("Head yaws: m1=%.3f, m2=%.3f, m3=%.3f", 
//              best_markers[0].head_yaw * 180.0/M_PI,
//              best_markers[1].head_yaw * 180.0/M_PI,
//              best_markers[2].head_yaw * 180.0/M_PI);
    
//     // Verify: if all head yaws are same, new method should match old method
//     if (std::abs(best_markers[0].head_yaw - best_markers[1].head_yaw) < 0.01 &&
//         std::abs(best_markers[1].head_yaw - best_markers[2].head_yaw) < 0.01) {
//         ROS_INFO("All markers from same view - angles should match between methods");
//     }

//     // Triangulate robot position
//     double xr, yr;
//         bool tri = triangulateRobotPosition(x1, y1, x2, y2, x3, y3, alpha1, alpha2, xr, yr);
//         if (!tri) {
//             ROS_WARN("Triangulation failed to find a valid robot position");
//             return false;
//         }

//     if (verbose_) {
//         ROS_INFO("Selected robot position: (%.3f, %.3f)", xr, yr);
//     }

//     // Compute robot orientation using the first marker
//     double robot_theta = computeYaw(best_markers[0].center, x1, y1, xr, yr);
//     if (verbose_) {
//         ROS_INFO("Computed robot orientation: %.3f rad (%.1f deg)", robot_theta, robot_theta * 180.0 / M_PI);
//     }

//     // Update pose
//     baseline_pose_.x = xr;
//     baseline_pose_.y = yr;
//     baseline_pose_.theta = robot_theta;
//     current_pose_ = baseline_pose_;
//     last_absolute_pose_time_ = ros::Time::now();
//     last_odom_pose_ = current_pose_;

//     // Update odom pose
//     initial_robot_x = xr;  
//     initial_robot_y = yr;
//     initial_robot_theta = robot_theta;
//     adjustment_x_ = initial_robot_x - odom_x_;
//     adjustment_y_ = initial_robot_y - odom_y_;
//     adjustment_theta_ = initial_robot_theta - odom_theta_;

//     ROS_INFO("ROBOT POSE: x = %.3f, y = %.3f, theta = %.3f degrees", xr, yr, robot_theta * 180.0 / M_PI);

//     // Publish marker visualization
//     if (!latest_image_.empty()) {
//         cv::Mat output_image = latest_image_.clone();
//         std::vector<int> ids;
//         std::vector<std::vector<cv::Point2f>> corners;
//         for (const auto& m : best_markers) {
//             ids.push_back(m.id);
//             corners.push_back(m.corners);
//         }
//         if (!ids.empty()) {
//             cv::aruco::drawDetectedMarkers(output_image, corners, ids);
//         }
//         sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
//         image_pub_.publish(img_msg);
//     }

//     is_scanning_ = false;
//     ROS_INFO("Absolute localization successful!");
//     return true;
// }

bool RobotLocalizationNode::computeAbsolutePoseWithActiveScanning() {
    if (!enable_active_scanning_) {
        return computeAbsolutePose();
    }

    if (projected_landmarks_.empty()) {
        ROS_WARN("No landmarks loaded");
        return false;
    }
    if (!camera_info_received_) {
        ROS_WARN("Camera intrinsics not received");
        return false;
    }

    // Set scanning flag
    is_scanning_ = true;
    initial_head_yaw_ = head_yaw_;

    // Clean up old markers
    cleanupOldMarkers();

    // Detect markers in current view
    detectAndStoreMarkers(head_yaw_);

    if (verbose_) {
        ROS_INFO("Marker memory size after current view: %zu", marker_memory_.size());
    }

    // If we don't have 3+ markers, scan for more
    if (marker_memory_.size() < 3) {
        int markers_needed = 3 - marker_memory_.size();
        ROS_INFO("Only found %zu markers in current view. Scanning for %d more...",
                 marker_memory_.size(), markers_needed);

        // Scan through predefined positions
        for (double scan_yaw : scan_positions_) {
            if (std::abs(scan_yaw - initial_head_yaw_) < 0.1) {
                continue;
            }

            if (!moveHeadToPosition(scan_yaw, 0.0)) {
                ROS_WARN("Failed to move head to scan position %.2f", scan_yaw);
                continue;
            }

            ros::spinOnce();
            detectAndStoreMarkers(scan_yaw);

            if (verbose_) {
                ROS_INFO("Marker memory size after scanning yaw=%.2f: %zu", scan_yaw, marker_memory_.size());
            }

            if (marker_memory_.size() >= 3) {
                ROS_INFO("Found sufficient markers (%zu) after scanning", marker_memory_.size());
                break;
            }
        }

        // Restore head to initial position
        moveHeadToPosition(initial_head_yaw_, 0.0);
    }

    // Check if we have enough markers
    if (marker_memory_.size() < 3) {
        ROS_WARN("Active scanning complete. Only found %zu markers (need 3)", marker_memory_.size());
        is_scanning_ = false;
        return false;
    }

    // Filter to only known landmarks
    std::vector<DetectedMarker> valid_markers;
    for (const auto& m : marker_memory_) {
        if (projected_landmarks_.find(m.id) != projected_landmarks_.end()) {
            valid_markers.push_back(m);
        }
    }

    if (valid_markers.size() < 3) {
        ROS_WARN("Could not find 3 valid markers from memory");
        is_scanning_ = false;
        return false;
    }

    // Try all possible orderings of the first 3 valid markers
    std::vector<int> indices = {0, 1, 2};
    
    struct TriangulationResult {
        bool success;
        double x, y, theta;
        double score;
        std::vector<int> marker_ids;
    };
    
    std::vector<TriangulationResult> results;
    
    // Try all 6 permutations
    do {
        DetectedMarker m1 = valid_markers[indices[0]];
        DetectedMarker m2 = valid_markers[indices[1]];
        DetectedMarker m3 = valid_markers[indices[2]];
        
        int id1 = m1.id, id2 = m2.id, id3 = m3.id;
        double x1 = projected_landmarks_[id1].first;
        double y1 = projected_landmarks_[id1].second;
        double x2 = projected_landmarks_[id2].first;
        double y2 = projected_landmarks_[id2].second;
        double x3 = projected_landmarks_[id3].first;
        double y3 = projected_landmarks_[id3].second;
        
        // Check landmark geometry
        double landmark_area = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0;
        if (landmark_area < 0.5) continue;
        
        // Compute angles
        double alpha1 = computeAngleWithHeadYaw(m1, m2);
        double alpha2 = computeAngleWithHeadYaw(m2, m3);
        
        // Check angle validity
        if (alpha1 < 5.0 || alpha2 < 5.0 || alpha1 > 120.0 || alpha2 > 120.0) {
            continue;
        }
        
        // Triangulate
        double xr, yr;
        if (!triangulateRobotPosition(x1, y1, x2, y2, x3, y3, alpha1, alpha2, xr, yr)) {
            continue;
        }
        
        // Sanity check distances
        double d1 = std::hypot(xr-x1, yr-y1);
        double d2 = std::hypot(xr-x2, yr-y2);
        double d3 = std::hypot(xr-x3, yr-y3);
        double max_dist = std::max({d1, d2, d3});
        
        if (max_dist > 20.0) continue;
        
        // Compute orientation
        double theta = computeYaw(m1.center, x1, y1, xr, yr);
        
        // Score based on geometry quality
        double avg_dist = (d1 + d2 + d3) / 3.0;
        double dist_score = 1.0 / (1.0 + std::abs(avg_dist - 5.0));
        double angle_balance = 1.0 - std::abs(alpha1 - alpha2) / 60.0;
        double area_score = std::min(landmark_area / 5.0, 1.0);

        double odom_score = 1.0;
        if (first_odom_received_) {
            // Compute current odometry-based estimate
            double odom_est_x = odom_x_ + adjustment_x_;
            double odom_est_y = odom_y_ + adjustment_y_;
            
            // Distance from odometry estimate
            double odom_dist = std::hypot(xr - odom_est_x, yr - odom_est_y);
            
            // Score: heavily penalize results far from odometry
            // Within 0.5m: full score, beyond 2m: very low score
            if (odom_dist < 0.5) {
                odom_score = 1.0;
            } else if (odom_dist < 1.0) {
                odom_score = 0.8;
            } else if (odom_dist < 2.0) {
                odom_score = 0.3;
            } else {
                odom_score = 0.05; // Very low but not zero
            }
            
            if (verbose_) {
                ROS_INFO("Candidate at (%.3f, %.3f) vs odom (%.3f, %.3f), dist=%.3f, odom_score=%.3f",
                        xr, yr, odom_est_x, odom_est_y, odom_dist, odom_score);
            }
        }

        double score = dist_score * angle_balance * area_score * odom_score;
        
        results.push_back({true, xr, yr, theta, score, {id1, id2, id3}});
        
    } while (std::next_permutation(indices.begin(), indices.end()));
    
    if (results.empty()) {
        ROS_WARN("No valid triangulation found for any marker ordering");
        is_scanning_ = false;
        return false;
    }
    
    // Sort by score and pick best
    std::sort(results.begin(), results.end(), 
        [](const TriangulationResult& a, const TriangulationResult& b) {
            return a.score > b.score;
        });
    
    auto& best = results[0];
    
    ROS_INFO("Using found ArUco markers:");
    ROS_INFO("Marker 1: ID %d at (%.3f, %.3f)", best.marker_ids[0], 
             projected_landmarks_[best.marker_ids[0]].first, 
             projected_landmarks_[best.marker_ids[0]].second);
    ROS_INFO("Marker 2: ID %d at (%.3f, %.3f)", best.marker_ids[1],
             projected_landmarks_[best.marker_ids[1]].first,
             projected_landmarks_[best.marker_ids[1]].second);
    ROS_INFO("Marker 3: ID %d at (%.3f, %.3f)", best.marker_ids[2],
             projected_landmarks_[best.marker_ids[2]].first,
             projected_landmarks_[best.marker_ids[2]].second);
    
    // Update pose
    baseline_pose_.x = best.x;
    baseline_pose_.y = best.y;
    baseline_pose_.theta = best.theta;
    current_pose_ = baseline_pose_;
    last_absolute_pose_time_ = ros::Time::now();
    
    // Update odom
    initial_robot_x = best.x;
    initial_robot_y = best.y;
    initial_robot_theta = best.theta;
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;
    
    ROS_INFO("ROBOT POSE: x = %.3f, y = %.3f, theta = %.3f degrees", 
             best.x, best.y, best.theta * 180.0 / M_PI);
    
    is_scanning_ = false;
    ROS_INFO("Absolute localization successful!");
    return true;
}

bool RobotLocalizationNode::computeAbsolutePose() {
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
            ROS_WARN("No image available for absolute robot localization");
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

        // Sort by position using indices
        if (!marker_ids.empty()) {
            // Create index vector [0, 1, 2, 3, ...]
            std::vector<size_t> indices(marker_ids.size());
            std::iota(indices.begin(), indices.end(), 0);
            
            // Sort indices by marker center x-coordinate
            std::sort(indices.begin(), indices.end(), 
                    [&marker_corners, &marker_ids](size_t a, size_t b) {
                        float center_a_x = (marker_corners[a][0].x + marker_corners[a][1].x + 
                                            marker_corners[a][2].x + marker_corners[a][3].x) / 4.0f;
                        float center_b_x = (marker_corners[b][0].x + marker_corners[b][1].x + 
                                            marker_corners[b][2].x + marker_corners[b][3].x) / 4.0f;
                        
                        // Primary sort: x-coordinate  
                        if (std::abs(center_a_x - center_b_x) > 1.0f) {
                            return center_a_x < center_b_x;
                        }
                        
                        // Secondary sort: marker ID (for consistent ordering)
                        return marker_ids[a] < marker_ids[b];
                    });
            
            // Create sorted copies using the sorted indices
            std::vector<int> sorted_ids;
            std::vector<std::vector<cv::Point2f>> sorted_corners;
            
            for (size_t idx : indices) {
                sorted_ids.push_back(marker_ids[idx]);
                sorted_corners.push_back(marker_corners[idx]);
            }
            
            // Replace original vectors
            marker_ids = std::move(sorted_ids);
            marker_corners = std::move(sorted_corners);
        }

        // Print detected markers
        if (!marker_ids.empty()) {
            if (verbose_) {
                ROS_INFO("Detected markers:");
            }
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                std::stringstream ss;
                if (verbose_) {
                    ss << "Marker " << marker_ids[i] << " corners: ";
                }
                for (size_t j = 0; j < marker_corners[i].size(); ++j) {
                    if (verbose_) {
                        ss << "(" << marker_corners[i][j].x << "," << marker_corners[i][j].y << ")";
                    }
                    if (j < marker_corners[i].size() - 1) ss << " ";
                }
                if (verbose_) {
                    ROS_INFO("%s", ss.str().c_str());
                }
            }
        } else {
            ROS_INFO("No markers detected");
        }

        // Compute marker centers
        std::vector<std::pair<double, double>> marker_centers;
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            auto& corners = marker_corners[i];
            double cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
            double cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;
            marker_centers.push_back({cx, cy});
        }

        // Assume first three detected markers are used
        int id1 = marker_ids[0], id2 = marker_ids[1], id3 = marker_ids[2];
        if (projected_landmarks_.find(id1) == projected_landmarks_.end() || 
            projected_landmarks_.find(id2) == projected_landmarks_.end() || 
            projected_landmarks_.find(id3) == projected_landmarks_.end()) {
            ROS_WARN("Unknown marker IDs detected: %d, %d, %d", id1, id2, id3);
            return false;
        }

        double x1 = projected_landmarks_[id1].first, y1 = projected_landmarks_[id1].second;
        double x2 = projected_landmarks_[id2].first, y2 = projected_landmarks_[id2].second;
        double x3 = projected_landmarks_[id3].first, y3 = projected_landmarks_[id3].second;

    ROS_INFO("Using markers from active scanning:");
    ROS_INFO("Marker 1: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id1, x1, y1, best_markers[0].head_yaw);
    ROS_INFO("Marker 2: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id2, x2, y2, best_markers[1].head_yaw);
    ROS_INFO("Marker 3: ID %d at (%.3f, %.3f) [head_yaw=%.2f]", id3, x3, y3, best_markers[2].head_yaw);

        // Check for collinear markers and small landmark triangle area
        double landmark_triangle_area = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / 2.0;
        if (landmark_triangle_area < 0.5) {
            ROS_WARN("Landmark triangle area too small (%.3f), rejecting configuration", landmark_triangle_area);
            return false;
        }
        double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        if (std::abs(cross_product) < 0.01) {
            ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
            return false;
        }

        // Compute angles alpha1 (between markers 1 and 2) and alpha2 (between markers 2 and 3)
        double alpha1 = computeAngle(marker_centers[0], marker_centers[1]);
        double alpha2 = computeAngle(marker_centers[1], marker_centers[2]);

        // Reject extreme angles
        if (alpha1 < 5.0 || alpha2 < 5.0 || alpha1 > 120.0 || alpha2 > 120.0) {
            ROS_WARN("Angles too extreme (alpha1=%.3f, alpha2=%.3f), rejecting configuration", alpha1, alpha2);
            return false;
        }

    if (verbose_) {
        ROS_INFO("Computed angles: alpha1=%.3f, alpha2=%.3f", alpha1, alpha2);
    }

    // Continue with triangulation (same as original computeAbsolutePose)
    // Convert angles to radians for calculation
    alpha1 = alpha1 * M_PI / 180.0;
    alpha2 = alpha2 * M_PI / 180.0;

    // Compute possible robot positions using circle-circle intersection
    // This is the same triangulation algorithm as the original implementation

    // First circle center options (between markers 1 and 2)
    double xc1_1, yc1_1, xc1_2, yc1_2, r1;
    int result1 = circle_centre(x1, y1, x2, y2, alpha1, &xc1_1, &yc1_1, &xc1_2, &yc1_2, &r1);

    if (result1 != 1) {
        ROS_WARN("Failed to compute circle centers for markers 1-2");
        is_scanning_ = false;
        return false;
    }

        // Compute robot orientation(yaw) using first marker
        double theta = computeYaw(marker_centers[0], x1, y1, xr, yr);

        // Update pose
        baseline_pose_.x = xr;
        baseline_pose_.y = yr;
        baseline_pose_.theta = theta; // Radians
        current_pose_ = baseline_pose_;
        last_absolute_pose_time_ = ros::Time::now();

        // Update odom pose
        initial_robot_x = xr;
        initial_robot_y = yr;
        initial_robot_theta = theta;
        adjustment_x_ = initial_robot_x - odom_x_;
        adjustment_y_ = initial_robot_y - odom_y_;
        adjustment_theta_ = initial_robot_theta - odom_theta_;

        ROS_INFO("ROBOT POSE: x = %.3f, y = %.3f, theta = %.3f degrees", xr, yr, theta * 180.0 / M_PI);
        if (verbose_) {
            cv::imshow("ArUco Markers", output_image);
            cv::waitKey(1);
        }

        return true;
    }
}

bool RobotLocalizationNode::computeAbsolutePoseWithDepth() {
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

    // Score and select best solution (same scoring as original)
    double best_score = -1e9;
    std::pair<double, double> best_solution;

    for (const auto& sol : solutions) {
        double rx = sol.first;
        double ry = sol.second;

        double score = 0.0;

        // Check distances to all three markers
        double d1 = std::sqrt((rx - x1)*(rx - x1) + (ry - y1)*(ry - y1));
        double d2 = std::sqrt((rx - x2)*(rx - x2) + (ry - y2)*(ry - y2));
        double d3 = std::sqrt((rx - x3)*(rx - x3) + (ry - y3)*(ry - y3));

        // Prefer solutions where distances are reasonable (2-10m typically)
        if (d1 > 2.0 && d1 < 10.0) score += 10;
        if (d2 > 2.0 && d2 < 10.0) score += 10;
        if (d3 > 2.0 && d3 < 10.0) score += 10;

        // Penalize extreme distances
        if (d1 < 1.0 || d1 > 15.0) score -= 20;
        if (d2 < 1.0 || d2 > 15.0) score -= 20;
        if (d3 < 1.0 || d3 > 15.0) score -= 20;

        // Compute triangle area formed by robot and markers
        double area12 = std::abs((x2-rx)*(y1-ry) - (x1-rx)*(y2-ry)) / 2.0;
        double area23 = std::abs((x3-rx)*(y2-ry) - (x2-rx)*(y3-ry)) / 2.0;
        double area13 = std::abs((x3-rx)*(y1-ry) - (x1-rx)*(y3-ry)) / 2.0;

        // Prefer larger triangulation areas (more stable)
        score += area12 + area23 + area13;

        if (score > best_score) {
            best_score = score;
            best_solution = sol;
        }
    }

    double robot_x = best_solution.first;
    double robot_y = best_solution.second;

    if (verbose_) {
        ROS_INFO("Selected robot position: (%.3f, %.3f) with score %.3f", robot_x, robot_y, best_score);
    }

    if (markers.size() < 3) {
        ROS_WARN("Insufficient valid depth measurements");
        return false;
    }

    // Solve for (xr, yr) using three markers
    double x1 = std::get<1>(markers[0]), y1 = std::get<2>(markers[0]), d1 = std::get<3>(markers[0]);
    double x2 = std::get<1>(markers[1]), y2 = std::get<2>(markers[1]), d2 = std::get<3>(markers[1]);
    double x3 = std::get<1>(markers[2]), y3 = std::get<2>(markers[2]), d3 = std::get<3>(markers[2]);

    // Check for collinear markers
    double cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    if (std::abs(cross_product) < 0.01) {
        ROS_WARN("Markers are nearly collinear, triangulation may be inaccurate");
        return false;
    }

    // Intersection of two circles)
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

    if (verbose_) {
        ROS_INFO("Computed robot orientation: %.3f rad (%.1f deg)", robot_theta, robot_theta * 180.0 / M_PI);
    }

    // Update pose
    baseline_pose_.x = xr;
    baseline_pose_.y = yr;
    baseline_pose_.theta = theta; // Radians
    current_pose_ = baseline_pose_;
    last_absolute_pose_time_ = ros::Time::now();

    // Update odom pose
    initial_robot_x = xr;
    initial_robot_y = yr;
    initial_robot_theta = theta;
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;

    ROS_INFO("Robot Pose (Depth): x=%.3f, y=%.3f, theta=%.3f degrees", xr, yr, theta * 180.0 / M_PI);
    if (verbose_){
        cv::imshow("ArUco Markers", output_image);
        cv::waitKey(1);
    }

    is_scanning_ = false;
    ROS_INFO("Active scanning localization successful!");
    return true;
}

void RobotLocalizationNode::publishPose() {
    pose_pub_.publish(current_pose_);
}
