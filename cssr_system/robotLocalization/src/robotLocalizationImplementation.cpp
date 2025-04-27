#include "robotLocalization/robotLocalizationInterface.h"
#include <fstream>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <angles/angles.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>

// Implementation of RobotLocalization methods
RobotLocalization::RobotLocalization() {
    loadConfiguration();
    relative_pose.x = 0.0;
    relative_pose.y = 0.0;
    relative_pose.theta = 0.0;

    process_image = false;
    first_odom_received_ = false;
    adjustment_x_ = adjustment_y_ = adjustment_theta_ = 0.0;
    odom_x_ = odom_y_ = odom_theta_ = 0.0;
    initROS();
    initializePoseAdjustments();

    // Initialize ArUco marker detector
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    parameters_ = cv::aruco::DetectorParameters::create();

    // camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.58, 0.0, 633.34, 
    //                                                  0.0, 910.52, 387.16, 
    //                                                  0.0, 0.0, 1.0);
    // dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

    camera_matrix_ = (cv::Mat_<double>(3, 3) << 909.18, 0.0, 642.07, 
                                                     0.0, 908.58, 384.44, 
                                                     0.0, 0.0, 1.0);
    dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);


    last_update_time_ = ros::Time::now();
    last_x_ = last_y_ = last_theta_ = 0.0;  // Store last position to track movement
    movement_threshold_ = 5.5; // Minimum distance to move before updating
    update_interval_ = ros::Duration(5.0); // Update every 5 seconds

    // Map to store known marker positions in the LAB frame
    marker_positions = {
        {10, cv::Point3d(6.6, 1.25, 90.0)},
        {15, cv::Point3d(3.2, 3.85, 90.0)},
        {20, cv::Point3d(3.8, 0.1, 90.0)},
        {25, cv::Point3d(2.0, 0.02, 90.0)},
        {30, cv::Point3d(2.0, 3.8, 90.0)},
        {35, cv::Point3d(0.0, 2.4, 0.0)},
        {40, cv::Point3d(0.0, 4.2, 0.0)},
        {45, cv::Point3d(0.0, 6.0, 0.0)},
        {50, cv::Point3d(0.0, 7.2, 0.0)},
        {55, cv::Point3d(0.1, 9.84, -90.0)},
        {60, cv::Point3d(2.0, 9.94, -90.0)},
        {65, cv::Point3d(5.0, 9.94, -90.0)},
        {70, cv::Point3d(6.6, 9.84, -90.0)},
        {75, cv::Point3d(6.8, 7.2, 180.0)},
        {80, cv::Point3d(6.8, 5.1, 180.0)},
        {85, cv::Point3d(6.8, 3.0, 180.0)}
    };
}

void RobotLocalization::loadConfiguration() {
    // Load configuration file
    ros::NodeHandle private_nh("~");
    std::string config_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/config/robotLocalizationConfiguration.ini";
    std::ifstream infile(config_file);
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open configuration file: %s", config_file.c_str());
        ros::shutdown();
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        config_[key] = value;
    }
    infile.close();

    // Set configuration variables
    platform_ = config_["platform"];
    camera_ = config_["camera"];
    reset_interval_ = std::stoi(config_["resetInterval"]);
    robot_topics_file_ = config_["robotTopics"];
    simulator_topics_file_ = config_["simulatorTopics"];
    verbose_mode = (config_["verboseMode"] == "true");

    // Load topic names
    loadTopicNames();
}

void RobotLocalization::loadTopicNames() {
    std::string topics_file;
    if (platform_ == "robot")
        topics_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/data/" + robot_topics_file_;
    else
        topics_file = ros::package::getPath(ROS_PACKAGE_NAME) + "/robotLocalization/data/" + simulator_topics_file_;

    std::ifstream infile(topics_file.c_str());
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open topics file: %s", topics_file.c_str());
        ros::shutdown();
    }

    std::string line;
    while (std::getline(infile, line)) {
        if (line.empty() || line[0] == '#') continue;
        size_t pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        topics_[key] = value;
    }
    infile.close();
}

void RobotLocalization::initROS() {
    odom_sub = nh.subscribe(topics_["Odometry"], 10, &RobotLocalization::odomCallback, this);
    imu_sub = nh.subscribe(topics_["IMU"], 10, &RobotLocalization::imuCallback, this);
    joint_state_sub = nh.subscribe(topics_["HeadYaw"], 10, &RobotLocalization::jointStateCallback, this);

    image_transport::ImageTransport it(nh);
    camera_sub = it.subscribe(topics_[camera_], 1, &RobotLocalization::imageCallback, this);

    pose_pub_ = nh.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
    reset_service = nh.advertiseService("/robotLocalization/reset_pose", &RobotLocalization::resetPoseService, this);
}

void RobotLocalization::initializePoseAdjustments() {
    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = angles::normalize_angle(initial_robot_theta - odom_theta_);
}

void RobotLocalization::setInitialValues(double x, double y, double theta) {
    initial_robot_x = x;
    initial_robot_y = y;
    initial_robot_theta = theta;

    ros::Rate rate(10);
    while (!first_odom_received_ && ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    // **Ensure odometry data is up-to-date**
    ros::spinOnce();

    // Calculate adjustments between initial pose and current odometry
    adjustment_x_ = initial_robot_x - odom_x_;
    adjustment_y_ = initial_robot_y - odom_y_;
    adjustment_theta_ = initial_robot_theta - odom_theta_;
}

void RobotLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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

    if (verbose_mode) {
        ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f degrees", current_x, current_y, angles::to_degrees(current_theta));
    }
}

void RobotLocalization::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Process IMU data if needed
}

void RobotLocalization::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "HeadYaw") {
            head_yaw_angle_ = msg->position[i];
            break;
        }
    }
}

void RobotLocalization::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // try {
    //     cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //     current_image_ = cv_ptr_->image;

    //     if (process_image) {
    //         recognizeLandmarks();
    //         process_image = false;
    //     }
    // } catch (cv_bridge::Exception& e) {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    // }

    ros::Time now = ros::Time::now();

    // Check if the last update was too recent
    if (now - last_update_time_ < update_interval_) {
        ROS_INFO_THROTTLE(5, "Skipping update - time interval not reached.");
        return;
    }

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        recognizeLandmarks(image);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


// void RobotLocalization::recognizeLandmarks() {
//     relative_pose.x = 0.0;
//     relative_pose.y = 0.0;
//     relative_pose.theta = 0.0;
// }


void RobotLocalization::recognizeLandmarks(const cv::Mat& image) {

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

    if (marker_ids.empty()) {
        return; // No markers detected
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    std::vector<cv::Point3d> estimated_positions;
    std::vector<double> weights;

    for (size_t i = 0; i < marker_ids.size(); i++) {
        int marker_id = marker_ids[i];

        // Ensure marker has a known position in the LAB frame
        if (marker_positions.find(marker_id) == marker_positions.end()) {
            ROS_WARN("Marker ID %d is not in the lookup table. Skipping.", marker_id);
            continue;
        }

        // Known marker position in LAB frame
        double Xm = marker_positions[marker_id].x;
        double Ym = marker_positions[marker_id].y;
        double Theta_m = marker_positions[marker_id].z; // In degrees

        // Marker pose relative to camera
        double Xc = tvecs[i][0];  // Camera-relative X
        double Yc = tvecs[i][1];  // Camera-relative Y
        double Theta_c = rvecs[i][2] * (180.0 / M_PI); // Convert radians to degrees

        // **Convert marker’s pose to global LAB frame**
        double robot_x = Xm + cos(Theta_m * M_PI / 180.0) * Xc - sin(Theta_m * M_PI / 180.0) * Yc;
        double robot_y = Ym + sin(Theta_m * M_PI / 180.0) * Xc + cos(Theta_m * M_PI / 180.0) * Yc;
        double robot_theta = Theta_m + Theta_c;

        // **Normalize theta within (-180, 180) degrees**
        robot_theta = fmod(robot_theta + 180.0, 360.0) - 180.0;

        // Store computed pose
        estimated_positions.push_back(cv::Point3d(robot_x, robot_y, robot_theta));

        // Compute weight (closer markers have higher influence)
        double distance = sqrt(Xc * Xc + Yc * Yc);
        double weight = 1.0 / (distance + 0.0001); // Avoid division by zero
        weights.push_back(weight);
    }

    double rotation_threshold_ = angles::from_degrees(90.0);

    if (estimated_positions.empty()) {
        ROS_WARN("No valid markers for absolute localization.");
        return;
    }

    // **Compute final estimated pose using weighted averaging**
    double sum_x = 0, sum_y = 0, sum_theta = 0, sum_weights = 0;
    for (size_t i = 0; i < estimated_positions.size(); i++) {
        sum_x += estimated_positions[i].x * weights[i];
        sum_y += estimated_positions[i].y * weights[i];
        sum_theta += estimated_positions[i].z * weights[i];
        sum_weights += weights[i];
    }

    double final_x = sum_x / sum_weights;
    double final_y = sum_y / sum_weights;
    double final_theta = sum_theta / sum_weights;

    // **Ensure pose updates only when robot has moved significantly**
    double distance_moved = sqrt(pow(final_x - last_x_, 2) + pow(final_y - last_y_, 2));
    double rotation_moved = fabs(final_theta - last_theta_);

    if (distance_moved < movement_threshold_ && rotation_moved < rotation_threshold_) {
        ROS_INFO_THROTTLE(5, "Skipping update - robot hasn't moved significantly.");
        return;
    }

    // **Update pose only if significant movement occurred**
    last_x_ = final_x;
    last_y_ = final_y;
    last_theta_ = final_theta;
    last_update_time_ = ros::Time::now();

    // **Publish updated pose**
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = final_x;
    pose_msg.pose.position.y = final_y;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(final_theta * M_PI / 180.0);
    pose_pub_.publish(pose_msg);

    ROS_INFO("Updated absolute pose: (%.2f, %.2f, %.2f)", final_x, final_y, final_theta);


    // std::vector<int> marker_ids;
    // std::vector<std::vector<cv::Point2f>> marker_corners;
    // cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

    // if (marker_ids.empty()) {
    //     return; // No markers detected
    // }

    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // std::vector<cv::Point3d> estimated_positions;
    // std::vector<double> weights;

    // for (size_t i = 0; i < marker_ids.size(); i++) {
    //     int marker_id = marker_ids[i];

    //     // Ensure marker has a known position in the LAB frame
    //     if (marker_positions.find(marker_id) == marker_positions.end()) {
    //         ROS_WARN("Marker ID %d is not in the lookup table. Skipping.", marker_id);
    //         continue;
    //     }

    //     // Known marker position in LAB frame
    //     double Xm = marker_positions[marker_id].x;
    //     double Ym = marker_positions[marker_id].y;
    //     double Theta_m = marker_positions[marker_id].z;

    //     // Marker pose relative to camera
    //     double Xc = tvecs[i][0]; 
    //     double Yc = tvecs[i][1]; 
    //     double Theta_c = rvecs[i][2];

    //     // Convert marker’s pose to global LAB frame
    //     double robot_x = Xm + cos(Theta_m * M_PI / 180.0) * Xc - sin(Theta_m * M_PI / 180.0) * Yc;
    //     double robot_y = Ym + sin(Theta_m * M_PI / 180.0) * Xc + cos(Theta_m * M_PI / 180.0) * Yc;
    //     double robot_theta = Theta_m + Theta_c * (180.0 / M_PI);

    //     // Store computed pose
    //     estimated_positions.push_back(cv::Point3d(robot_x, robot_y, robot_theta));

    //     // Compute weight (closer markers have higher influence)
    //     double distance = sqrt(Xc * Xc + Yc * Yc);
    //     double weight = 1.0 / (distance + 0.0001); // Avoid division by zero
    //     weights.push_back(weight);
    // }

    // if (estimated_positions.empty()) {
    //     ROS_WARN("No valid markers for absolute localization.");
    //     return;
    // }

    // double rotation_threshold_ = angles::from_degrees(90.0);

    // // Compute final estimated pose using weighted average
    // double sum_x = 0, sum_y = 0, sum_theta = 0, sum_weights = 0;
    // for (size_t i = 0; i < estimated_positions.size(); i++) {
    //     sum_x += estimated_positions[i].x * weights[i];
    //     sum_y += estimated_positions[i].y * weights[i];
    //     sum_theta += estimated_positions[i].z * weights[i];
    //     sum_weights += weights[i];
    // }

    // double final_x = sum_x / sum_weights;
    // double final_y = sum_y / sum_weights;
    // double final_theta = sum_theta / sum_weights;

    // // Check if the robot has moved significantly before updating
    // double distance_moved = sqrt(pow(final_x - last_x_, 2) + pow(final_y - last_y_, 2));
    // double rotation_moved = fabs(final_theta - last_theta_);

    // if (distance_moved < movement_threshold_ && rotation_moved < rotation_threshold_) {
    //     ROS_INFO_THROTTLE(5, "Skipping update - robot hasn't moved significantly.");
    //     return;
    // }

    // // Update pose only if significant movement occurred
    // last_x_ = final_x;
    // last_y_ = final_y;
    // last_theta_ = final_theta;
    // last_update_time_ = ros::Time::now();

    // // Publish updated pose
    // geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header.stamp = ros::Time::now();
    // pose_msg.header.frame_id = "map";
    // pose_msg.pose.position.x = final_x;
    // pose_msg.pose.position.y = final_y;
    // pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(final_theta * M_PI / 180.0);
    // pose_pub_.publish(pose_msg);

    // ROS_INFO("Updated absolute pose: (%.2f, %.2f, %.2f)", final_x, final_y, final_theta);

    // std::vector<int> marker_ids;
    // std::vector<std::vector<cv::Point2f>> marker_corners;
    // cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

    // if (marker_ids.empty()) {
    //     return;  // No markers detected
    // }

    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0;
    // int valid_markers = 0;

    // for (size_t i = 0; i < marker_ids.size(); i++) {
    //     int marker_id = marker_ids[i];

    //     // Ensure marker exists in lookup table
    //     if (marker_positions.find(marker_id) == marker_positions.end()) {
    //         ROS_WARN("Marker ID %d is not in the lookup table. Skipping.", marker_id);
    //         continue;
    //     }

    //     // Known marker position in LAB frame
    //     double Xm = marker_positions[marker_id].x;
    //     double Ym = marker_positions[marker_id].y;
    //     double Theta_m = marker_positions[marker_id].z;  // θ in degrees

    //     // Marker pose relative to camera
    //     double Xc = tvecs[i][0];  // X relative to camera
    //     double Yc = tvecs[i][1];  // Y relative to camera
    //     double Theta_c = rvecs[i][2] * (180.0 / M_PI);  // Convert to degrees

    //     // Compute the marker's actual position in LAB frame
    //     double marker_x_lab = Xm + cos(Theta_m * M_PI / 180.0) * Xc - sin(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_y_lab = Ym + sin(Theta_m * M_PI / 180.0) * Xc + cos(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_theta_lab = Theta_m + Theta_c;

    //     // Accumulate values for averaging
    //     sum_x += marker_x_lab;
    //     sum_y += marker_y_lab;
    //     sum_theta += marker_theta_lab;
    //     valid_markers++;
    // }

    // // Compute the final absolute pose using all detected markers
    // if (valid_markers > 0) {
    //     double robot_x = sum_x / valid_markers;
    //     double robot_y = sum_y / valid_markers;
    //     double robot_theta = sum_theta / valid_markers;

    //     // Check movement threshold before updating
    //     double distance_moved = sqrt(pow(robot_x - last_x_, 2) + pow(robot_y - last_y_, 2));
    //     if (distance_moved < movement_threshold_) {
    //         ROS_INFO_THROTTLE(5, "Skipping update - robot hasn't moved significantly.");
    //         return;
    //     }

    //     // Update last known position
    //     last_x_ = robot_x;
    //     last_y_ = robot_y;
    //     last_update_time_ = ros::Time::now();

    //     // Reset Pose Service
    //     req.x = robot_x;
    //     req.y = robot_y;
    //     req.theta = robot_theta;
    //     bool success = this->resetPoseService(req, res);

    //     if (success) {
    //         ROS_INFO("Pose reset with absolute localization using multiple markers. Success: %d", res.success);
    //         ROS_INFO("Final Computed Pose: (%.2f, %.2f, %.2f)", robot_x, robot_y, robot_theta);
    //     }
    // }

    // // Draw detected markers
    // cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
    // cv::imshow("Aruco Detection", image);
    // cv::waitKey(1);


    // std::vector<int> marker_ids;
    // std::vector<std::vector<cv::Point2f>> marker_corners;
    // cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

    // if (marker_ids.empty()) {
    //     ROS_WARN_THROTTLE(5, "No markers detected.");
    //     return;
    // }

    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // // ✅ Track the best marker for pose correction
    // int best_marker_id = -1;
    // double best_reprojection_error = std::numeric_limits<double>::max();
    // double best_robot_x = 0, best_robot_y = 0, best_robot_theta = 0;

    // for (size_t i = 0; i < marker_ids.size(); i++) {
    //     int marker_id = marker_ids[i];

    //     // ✅ Ensure marker is in the lookup table
    //     if (marker_positions.find(marker_id) == marker_positions.end()) {
    //         ROS_WARN("Marker ID %d is not in lookup table. Skipping.", marker_id);
    //         continue;
    //     }

    //     // ✅ Get the known marker position in LAB frame
    //     double Xm = marker_positions[marker_id].x;
    //     double Ym = marker_positions[marker_id].y;
    //     double Theta_m = marker_positions[marker_id].z; // Theta in degrees

    //     // ✅ Get detected marker pose (relative to camera)
    //     double Xc = tvecs[i][0];  // X relative to camera
    //     double Yc = tvecs[i][1];  // Y relative to camera
    //     double Theta_c = rvecs[i][2];  // Rotation (yaw)

    //     // ✅ Convert marker pose from camera frame → LAB frame
    //     double marker_x_lab = Xm + cos(Theta_m * M_PI / 180.0) * Xc - sin(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_y_lab = Ym + sin(Theta_m * M_PI / 180.0) * Xc + cos(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_theta_lab = Theta_m + Theta_c * (180.0 / M_PI);

    //     // ✅ Robot position is the same as the camera position
    //     double robot_x = marker_x_lab;
    //     double robot_y = marker_y_lab;
    //     double robot_theta = marker_theta_lab;

    //     // ✅ Compute reprojection error (to filter bad detections)
    //     double reprojection_error = fabs(Xc) + fabs(Yc); // Simple heuristic for now

    //     if (reprojection_error < best_reprojection_error) {
    //         best_reprojection_error = reprojection_error;
    //         best_marker_id = marker_id;
    //         best_robot_x = robot_x;
    //         best_robot_y = robot_y;
    //         best_robot_theta = robot_theta;
    //     }
    // }

    // // ✅ Only update pose if the robot has moved significantly
    // if (best_marker_id != -1) {
    //     double distance_moved = sqrt(pow(best_robot_x - last_x_, 2) + pow(best_robot_y - last_y_, 2));
    //     double rotation_moved = fabs(angles::normalize_angle(best_robot_theta - last_theta_));
    //     double rotation_threshold = angles::from_degrees(90.0);

    //     static bool has_updated = false;

    //     if (!has_updated || (distance_moved > movement_threshold_ || rotation_moved > rotation_threshold)) {
    //         has_updated = true;

    //         // ✅ Reset Pose Service
    //         req.x = best_robot_x;
    //         req.y = best_robot_y;
    //         req.theta = best_robot_theta;
    //         bool success = this->resetPoseService(req, res);

    //         if (success) {
    //             last_x_ = best_robot_x;
    //             last_y_ = best_robot_y;
    //             last_theta_ = best_robot_theta;
    //             last_update_time_ = ros::Time::now();

    //             ROS_INFO("Updated absolute pose using marker %d: (%.2f, %.2f, %.2f)", 
    //                      best_marker_id, best_robot_x, best_robot_y, best_robot_theta);
    //         }
    //     } else {
    //         ROS_INFO_THROTTLE(5, "Skipping update - robot has not moved significantly.");
    //     }
    // }

    // // ✅ Draw detected markers
    // cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
    // cv::imshow("Aruco Detection", image);
    // cv::waitKey(1);


    // std::vector<int> marker_ids;
    // std::vector<std::vector<cv::Point2f>> marker_corners;
    // cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);
    
    // if (marker_ids.empty()) {
    //     return; // No markers detected
    // }
    
    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
    
    // int best_marker_id = -1;
    // double best_distance = std::numeric_limits<double>::max();
    // double best_robot_x = 0, best_robot_y = 0, best_robot_theta = 0;
    // double rotation_moved;

    // for (size_t i = 0; i < marker_ids.size(); i++) {
    //     int marker_id = marker_ids[i];

    //     // Ensure marker has a known position in the LAB frame
    //     if (marker_positions.find(marker_id) == marker_positions.end()) {
    //         ROS_WARN("Marker ID %d is not in the lookup table. Skipping.", marker_id);
    //         continue;
    //     }

    //     // Get the known marker position in LAB frame
    //     double Xm = marker_positions[marker_id].x;
    //     double Ym = marker_positions[marker_id].y;
    //     double Theta_m = marker_positions[marker_id].z; // Theta in degrees

    //     // Get marker pose from camera
    //     double Xc = tvecs[i][0];  // Marker X relative to camera
    //     double Yc = tvecs[i][1];  // Marker Y relative to camera
    //     double Theta_c = rvecs[i][2];  // Rotation (yaw)

    //     // Convert marker's relative pose to global LAB frame
    //     double marker_x_lab = Xm + cos(Theta_m * M_PI / 180.0) * Xc - sin(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_y_lab = Ym + sin(Theta_m * M_PI / 180.0) * Xc + cos(Theta_m * M_PI / 180.0) * Yc;
    //     double marker_theta_lab = Theta_m + Theta_c * (180.0 / M_PI); // Convert radian to degree

    //     // Compute robot position (since camera is on robot, its pose = robot pose)
    //     double robot_x = marker_x_lab;
    //     double robot_y = marker_y_lab;
    //     double robot_theta = marker_theta_lab;


    //     // // Extract marker pose from camera (relative to camera)
    //     // double marker_x = tvecs[i][0];
    //     // double marker_y = tvecs[i][1];
    //     // double marker_theta = rvecs[i][2];
        
    //     // // Adjust pose based on known marker positions
    //     // double corrected_x = marker_x;
    //     // double corrected_y = marker_y;
    //     // double corrected_theta = marker_theta;
        
    //     // // Update the estimated pose
    //     // estimated_pose_x_ = corrected_x;
    //     // estimated_pose_y_ = corrected_y;
    //     // estimated_pose_theta_ = corrected_theta;

    //     // Check if the robot has moved significantly before updating
    //     double distance_moved = sqrt(pow(robot_x - last_x_, 2) + pow(robot_y - last_y_, 2));
    //     rotation_moved = fabs(robot_theta - last_theta_);  // Absolute difference in rotation

    //     if (distance_moved < best_distance) {
    //         best_distance = distance_moved;
    //         best_marker_id = marker_id;
    //         best_robot_x = robot_x;
    //         best_robot_y = robot_y;
    //         best_robot_theta = marker_theta_lab;
    //     }
    // }

    // // ✅ Only update if a marker is found AND if the robot has moved significantly
    // static bool has_updated = false;
    // double rotation_threshold = angles::from_degrees(90.0);
    // if (best_marker_id != -1) {

    //     // ✅ If this is the first update OR the robot has moved a certain distance, update
    //     if (!has_updated || (best_distance > movement_threshold_ || rotation_moved > rotation_threshold)) {
    //         has_updated = true;

    //         // ✅ Reset Pose Service
    //         req.x = best_robot_x;
    //         req.y = best_robot_y;
    //         req.theta = best_robot_theta;
    //         bool success = this->resetPoseService(req, res);

    //         if (success) {
    //             last_x_ = best_robot_x;
    //             last_y_ = best_robot_y;
    //             last_update_time_ = ros::Time::now();

    //             ROS_INFO("Updated absolute pose using marker %d: (%.2f, %.2f, %.2f)", 
    //                      best_marker_id, best_robot_x, best_robot_y, best_robot_theta);
    //         }
    //     } else {
    //         ROS_INFO_THROTTLE(5, "Skipping update - robot has not moved significantly.");
    //     }
    // }

    // // ✅ Draw detected markers
    // cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
    // cv::imshow("Aruco Detection", image);
    // cv::waitKey(1);


    //     // if (distance_moved < movement_threshold_) {
    //     //     ROS_INFO_THROTTLE(5, "Skipping update - robot hasn't moved significantly.");
    //     //     return;
    //     // }

    //     // Update last known position
    //     last_x_ = Xc;
    //     last_y_ = Yc;
    //     last_update_time_ = ros::Time::now();
        
    //     // // Publish updated pose
    //     // geometry_msgs::PoseStamped pose_msg;
    //     // pose_msg.header.stamp = ros::Time::now();
    //     // pose_msg.header.frame_id = "map";
    //     // pose_msg.pose.position.x = robot_x; // estimated_pose_x_;
    //     // pose_msg.pose.position.y = robot_y; // estimated_pose_y_;
    //     // pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(robot_theta * M_PI / 180.0);
    //     // pose_pub_.publish(pose_msg);
        
    //     // // ROS_INFO("Updated absolute pose using marker %d: (%.2f, %.2f, %.2f)", marker_ids[i], corrected_x, corrected_y, corrected_theta);
    //     // ROS_INFO("Updated absolute pose using marker %d: (%.2f, %.2f, %.2f)", marker_id, robot_x, robot_y, robot_theta);

    //     // Reset Pose Service
    //     req.x = robot_x;
    //     req.y = robot_y;
    //     req.theta = robot_theta;
    //     bool success = this->resetPoseService(req, res);
    //     if (success) {
    //         ROS_INFO("Pose reset with absolute localization. Success: %d", res.success);
    //         ROS_INFO("Marker %d: (%.2f, %.2f, %.2f)", marker_id, robot_x, robot_y, robot_theta);

    //     }
    // }
    cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
    cv::imshow("Aruco Detection", image);
    cv::waitKey(3);
}


bool RobotLocalization::resetPoseService(cssr_system::SetPose::Request& req, cssr_system::SetPose::Response& res) {
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


std::string RobotLocalization::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}




// #include "robotLocalization/robotLocalizationInterface.h"
// #include <fstream>
// #include <ros/package.h>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <angles/angles.h>

// RobotLocalization::RobotLocalization() : aruco_detected_(false) {
//     // Subscribe to odometry and ArUco marker pose topics
//     odom_sub_ = nh_.subscribe("/naoqi_driver/odom", 10, &RobotLocalization::odometryCallback, this);
//     aruco_sub_ = nh_.subscribe("/aruco_localization/pose", 10, &RobotLocalization::arucoPoseCallback, this);

//     // Publish estimated pose
//     pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_localization/pose", 10);

//     // Service to reset the pose
//     reset_pose_service_ = nh_.advertiseService("/robot_localization/reset_pose", &RobotLocalization::resetPoseService, this);

//     // Initialize current pose
//     current_pose_.setIdentity();
// }

// void RobotLocalization::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//     odom_x_ = msg->pose.pose.position.x;
//     odom_y_ = msg->pose.pose.position.y;
//     odom_theta_ = tf::getYaw(msg->pose.pose.orientation);

//     updateLocalization();
// }

// void RobotLocalization::arucoPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//     aruco_x_ = msg->pose.position.x;
//     aruco_y_ = msg->pose.position.y;
//     aruco_theta_ = tf::getYaw(msg->pose.orientation);
//     aruco_detected_ = true;

//     updateLocalization();
// }

// void RobotLocalization::updateLocalization() {
//     tf::Transform odom_transform;
//     odom_transform.setOrigin(tf::Vector3(odom_x_, odom_y_, 0.0));
//     tf::Quaternion odom_q;
//     odom_q.setRPY(0, 0, odom_theta_);
//     odom_transform.setRotation(odom_q);

//     if (aruco_detected_) {
//         tf::Transform aruco_transform;
//         aruco_transform.setOrigin(tf::Vector3(aruco_x_, aruco_y_, 0.0));
//         tf::Quaternion aruco_q;
//         aruco_q.setRPY(0, 0, aruco_theta_);
//         aruco_transform.setRotation(aruco_q);

//         // Blend odometry and ArUco localization (Kalman filter can be added)
//         tf::Vector3 interpolated_position = current_pose_.getOrigin().lerp(aruco_transform.getOrigin(), 0.7);
//         tf::Quaternion interpolated_rotation = current_pose_.getRotation().slerp(aruco_transform.getRotation(), 0.7);

//         current_pose_.setOrigin(interpolated_position);
//         current_pose_.setRotation(interpolated_rotation);
//     } else {
//         current_pose_ = odom_transform;
//     }

//     publishPose();
// }

// bool RobotLocalization::resetPoseService(cssr_system::SetPose::Request &req, cssr_system::SetPose::Response &res) {
//     current_pose_.setOrigin(tf::Vector3(req.x, req.y, 0.0));
//     tf::Quaternion q;
//     q.setRPY(0, 0, req.theta);
//     current_pose_.setRotation(q);

//     ROS_INFO("Pose reset to x: %f, y: %f, theta: %f", req.x, req.y, req.theta);
//     res.success = true;
//     return true;
// }

// void RobotLocalization::publishPose() {
//     geometry_msgs::PoseStamped pose_msg;
//     pose_msg.header.stamp = ros::Time::now();
//     pose_msg.header.frame_id = "map";

//     pose_msg.pose.position.x = current_pose_.getOrigin().x();
//     pose_msg.pose.position.y = current_pose_.getOrigin().y();
//     pose_msg.pose.position.z = 0.0;
//     tf::quaternionTFToMsg(current_pose_.getRotation(), pose_msg.pose.orientation);

//     pose_pub_.publish(pose_msg);
//     tf_broadcaster_.sendTransform(tf::StampedTransform(current_pose_, ros::Time::now(), "map", "base_footprint"));
// }
