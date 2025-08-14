#include "pepperLocalization/pepperLocalizationInterface.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

PepperArucoLocalizer::PepperArucoLocalizer() 
    : nh_(), pnh_("~"), it_(nh_), tf_listener_(tf_buffer_), has_last_pose_(false) {
    
    // Initialize ArUco detector
    aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    detector_params_ = cv::aruco::DetectorParameters::create();
    
    // Load parameters
    loadCameraParameters();
    loadMarkerPositions();
    
    // ROS parameters
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("world_frame", world_frame_, "map");
    pnh_.param<double>("marker_size", marker_size_, 0.2);
    
    // Camera offset parameters
    pnh_.param<double>("camera_offset_x", camera_offset_[0], 0.0);
    pnh_.param<double>("camera_offset_y", camera_offset_[1], 0.0);
    pnh_.param<double>("camera_offset_z", camera_offset_[2], 1.2);

    pnh_.param<double>("filter_alpha", filter_alpha_, 0.3);
    
    // ROS subscribers and publishers
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, 
                              &PepperArucoLocalizer::imageCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    
    ROS_INFO("Pepper ArUco Localizer initialized");
}

PepperArucoLocalizer::~PepperArucoLocalizer() {
}

void PepperArucoLocalizer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } else {
            ROS_ERROR("Unsupported image encoding: %s. Expected RGB8 or BGR8", msg->encoding.c_str());
            return;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    
    if (detectMarkers(cv_ptr->image, ids, corners)) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, 
                                           camera_matrix_, dist_coeffs_, 
                                           rvecs, tvecs);
        
        // Debug info
        for (size_t i = 0; i < ids.size(); ++i) {
            ROS_INFO("Detected marker %d at distance %.2f meters", ids[i], cv::norm(tvecs[i]));
        }
        
        // Compute robot pose
        RobotPose pose = computeRobotPose(rvecs, tvecs, ids);
        
        if (pose.valid) {
            // Apply temporal filtering if you have a previous pose
            if (has_last_pose_) {
                // Smooth position
                pose.x = filter_alpha_ * pose.x + (1.0 - filter_alpha_) * last_valid_pose_.x;
                pose.y = filter_alpha_ * pose.y + (1.0 - filter_alpha_) * last_valid_pose_.y;
                
                // Smooth yaw (handle angle wrapping)
                double yaw_diff = pose.yaw - last_valid_pose_.yaw;
                while (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2*M_PI;
                pose.yaw = last_valid_pose_.yaw + filter_alpha_ * yaw_diff;
            }
            
            last_valid_pose_ = pose;
            has_last_pose_ = true;
            
            publishFullPose(pose, msg->header);
            broadcastFullTransform(pose, msg->header);
            
            // CORRECTED LINE - use ids.size() instead of robot_positions.size()
            ROS_INFO("Final robot pose: (%.2f, %.2f, %.1f°) from %zu detected markers", 
                     pose.x, pose.y, pose.yaw * 180.0 / M_PI, ids.size());
        } else {
            ROS_WARN("No valid markers detected for localization");
        }
    }
}

bool PepperArucoLocalizer::detectMarkers(const cv::Mat& image, 
                                        std::vector<int>& ids,
                                        std::vector<std::vector<cv::Point2f>>& corners) {
    cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, detector_params_);
    return !ids.empty();
}

double PepperArucoLocalizer::computeYawFromMarker(const cv::Vec3d& rvec, 
                                                 const cv::Vec3d& tvec,
                                                 const MarkerInfo& marker_info) {
    // Convert rotation vector to rotation matrix
    cv::Mat R_marker_to_camera;
    cv::Rodrigues(rvec, R_marker_to_camera);
    
    // Camera to marker transformation
    cv::Mat R_camera_to_marker = R_marker_to_camera.t();
    
    // Known marker orientation in world frame
    cv::Mat R_world_to_marker = eulerToRotationMatrix(marker_info.rotation);
    
    // Camera orientation in world frame
    cv::Mat R_world_to_camera = R_world_to_marker * R_camera_to_marker;
    
    // Extract yaw from rotation matrix
    double yaw = atan2(R_world_to_camera.at<double>(1, 0), 
                       R_world_to_camera.at<double>(0, 0));
    
    // Debug: Convert to degrees and log
    double yaw_degrees = yaw * 180.0 / M_PI;
    ROS_DEBUG("Marker yaw computation: %.1f degrees", yaw_degrees);
    
    return yaw;
}

// Add this method to compute full pose
RobotPose PepperArucoLocalizer::computeRobotPose(const std::vector<cv::Vec3d>& rvecs,
                                                const std::vector<cv::Vec3d>& tvecs,
                                                const std::vector<int>& ids) {
    std::vector<cv::Point3d> robot_positions;
    std::vector<double> yaw_estimates;
    std::vector<int> valid_marker_ids;
    
    for (size_t i = 0; i < ids.size(); ++i) {
        auto marker_it = known_markers_.find(ids[i]);
        if (marker_it == known_markers_.end()) {
            ROS_DEBUG("Marker %d not in known markers list, skipping", ids[i]);
            continue;
        }
        
        const MarkerInfo& marker = marker_it->second;
        
        // Convert rotation vector to rotation matrix
        cv::Mat R_marker_to_camera;
        cv::Rodrigues(rvecs[i], R_marker_to_camera);
        
        // Check if detection is reasonable
        double distance = cv::norm(tvecs[i]);
        ROS_INFO("Processing marker %d: known=true, distance=%.2f", ids[i], distance);
        
        if (distance > 15.0 || distance < 0.1) {
            ROS_DEBUG("Marker %d: unreasonable distance %.2f, skipping", ids[i], distance);
            continue;
        }
        
        // Camera position relative to marker
        cv::Mat t_camera_to_marker = -R_marker_to_camera.t() * cv::Mat(tvecs[i]);
        
        // Transform camera position to world coordinates
        cv::Point3d camera_world_pos;
        camera_world_pos.x = marker.position[0] + t_camera_to_marker.at<double>(0);
        camera_world_pos.y = marker.position[1] + t_camera_to_marker.at<double>(1);
        camera_world_pos.z = marker.position[2] + t_camera_to_marker.at<double>(2);
        
        // DECLARE robot_world_pos BEFORE using it in debug statements
        cv::Point3d robot_world_pos;
        
        // TEST: Disable camera offset temporarily
        robot_world_pos.x = camera_world_pos.x + 3.0; // - camera_offset_[0];
        robot_world_pos.y = camera_world_pos.y + 4.5; // - camera_offset_[1]; 
        robot_world_pos.z = camera_world_pos.z; // - camera_offset_[2];
        
        // NOW you can use robot_world_pos in debug statements
        ROS_INFO("Marker %d: camera_world_pos (%.2f, %.2f, %.2f) -> robot_pos (%.2f, %.2f)", 
                 ids[i], camera_world_pos.x, camera_world_pos.y, camera_world_pos.z,
                 robot_world_pos.x, robot_world_pos.y);
        
        ROS_INFO("Marker %d: computed robot pos (%.2f, %.2f)", ids[i], robot_world_pos.x, robot_world_pos.y);
        
        // Expanded bounds checking - should now include actual robot position (4.4, 7.8)
        if (robot_world_pos.x < 0.0 || robot_world_pos.x > 8.0 ||    // 0-8m X range
            robot_world_pos.y < 0.0 || robot_world_pos.y > 12.0) {   // 0-12m Y range
            ROS_WARN("Marker %d: robot position (%.2f, %.2f) outside lab bounds, REJECTING", 
                    ids[i], robot_world_pos.x, robot_world_pos.y);
            continue;
        }
        
        robot_positions.push_back(robot_world_pos);
        valid_marker_ids.push_back(ids[i]);
        
        // Compute yaw
        double yaw = computeYawFromMarker(rvecs[i], tvecs[i], marker);
        yaw_estimates.push_back(yaw);
    }
    
    RobotPose pose;
    pose.valid = false;
    
    if (robot_positions.size() >= 1) {
        // If we have multiple positions, do outlier rejection
        if (robot_positions.size() >= 3) {
            // Calculate median position
            std::vector<double> x_vals, y_vals;
            for (const auto& pos : robot_positions) {
                x_vals.push_back(pos.x);
                y_vals.push_back(pos.y);
            }
            
            std::sort(x_vals.begin(), x_vals.end());
            std::sort(y_vals.begin(), y_vals.end());
            
            double median_x = x_vals[x_vals.size() / 2];
            double median_y = y_vals[y_vals.size() / 2];
            
            // Only keep positions within 2 meters of median
            std::vector<cv::Point3d> filtered_positions;
            std::vector<double> filtered_yaws;
            std::vector<int> filtered_ids;
            
            for (size_t i = 0; i < robot_positions.size(); ++i) {
                double dx = robot_positions[i].x - median_x;
                double dy = robot_positions[i].y - median_y;
                double dist_from_median = sqrt(dx*dx + dy*dy);
                
                if (dist_from_median < 2.0) {  // Within 2 meters of median
                    filtered_positions.push_back(robot_positions[i]);
                    filtered_yaws.push_back(yaw_estimates[i]);
                    filtered_ids.push_back(valid_marker_ids[i]);
                } else {
                    ROS_WARN("Marker %d: position (%.2f, %.2f) is %.2f meters from median, REJECTING", 
                             valid_marker_ids[i], robot_positions[i].x, robot_positions[i].y, dist_from_median);
                }
            }
            
            robot_positions = filtered_positions;
            yaw_estimates = filtered_yaws;
            valid_marker_ids = filtered_ids;
        }
        
        if (!robot_positions.empty()) {
            // Average the remaining positions
            pose.x = 0.0; pose.y = 0.0;
            for (const auto& pos : robot_positions) {
                pose.x += pos.x;
                pose.y += pos.y;
            }
            pose.x /= robot_positions.size();
            pose.y /= robot_positions.size();
            
            // Average yaw estimates (handle angle wrapping)
            double sum_sin = 0.0, sum_cos = 0.0;
            for (double yaw : yaw_estimates) {
                sum_sin += sin(yaw);
                sum_cos += cos(yaw);
            }
            pose.yaw = atan2(sum_sin, sum_cos);
            
            pose.valid = true;
            
            ROS_INFO("Accepted %zu markers for pose computation (IDs: ", robot_positions.size());
            for (int id : valid_marker_ids) {
                ROS_INFO("  %d", id);
            }
        }
    }
    
    return pose;
}

cv::Mat PepperArucoLocalizer::eulerToRotationMatrix(const cv::Vec3d& euler) {
    double roll = euler[0], pitch = euler[1], yaw = euler[2];
    
    cv::Mat Rx = cv::Mat::zeros(3, 3, CV_64F);
    Rx.at<double>(0,0) = 1;
    Rx.at<double>(1,1) = cos(roll);
    Rx.at<double>(1,2) = -sin(roll);
    Rx.at<double>(2,1) = sin(roll);
    Rx.at<double>(2,2) = cos(roll);
    
    cv::Mat Ry = cv::Mat::zeros(3, 3, CV_64F);
    Ry.at<double>(0,0) = cos(pitch);
    Ry.at<double>(0,2) = sin(pitch);
    Ry.at<double>(1,1) = 1;
    Ry.at<double>(2,0) = -sin(pitch);
    Ry.at<double>(2,2) = cos(pitch);
    
    cv::Mat Rz = cv::Mat::zeros(3, 3, CV_64F);
    Rz.at<double>(0,0) = cos(yaw);
    Rz.at<double>(0,1) = -sin(yaw);
    Rz.at<double>(1,0) = sin(yaw);
    Rz.at<double>(1,1) = cos(yaw);
    Rz.at<double>(2,2) = 1;
    
    return Rz * Ry * Rx;
}

void PepperArucoLocalizer::publishPose(double yaw, const std_msgs::Header& header) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.header.frame_id = world_frame_;
    
    // For now, only publishing yaw. You can extend this to include x, y position
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    
    pose_msg.pose.orientation = tf2::toMsg(q);
    
    pose_pub_.publish(pose_msg);
}

void PepperArucoLocalizer::broadcastTransform(double yaw, const std_msgs::Header& header) {
    geometry_msgs::TransformStamped transform_stamped;
    
    transform_stamped.header = header;
    transform_stamped.header.frame_id = world_frame_;
    transform_stamped.child_frame_id = base_frame_;
    
    // Position (you might want to compute this from markers too)
    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;
    
    // Orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_.sendTransform(transform_stamped);
}

void PepperArucoLocalizer::publishFullPose(const RobotPose& pose, const std_msgs::Header& header) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = header;
    pose_msg.header.frame_id = world_frame_;
    
    // Set position
    pose_msg.pose.position.x = pose.x;
    pose_msg.pose.position.y = pose.y;
    pose_msg.pose.position.z = 0.0;  // Assuming 2D localization
    
    // Set orientation (yaw only)
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    pose_msg.pose.orientation = tf2::toMsg(q);
    
    pose_pub_.publish(pose_msg);
}

void PepperArucoLocalizer::broadcastFullTransform(const RobotPose& pose, const std_msgs::Header& header) {
    geometry_msgs::TransformStamped transform_stamped;
    
    transform_stamped.header = header;
    transform_stamped.header.frame_id = world_frame_;
    transform_stamped.child_frame_id = base_frame_;
    
    // Set position
    transform_stamped.transform.translation.x = pose.x;
    transform_stamped.transform.translation.y = pose.y;
    transform_stamped.transform.translation.z = 0.0;
    
    // Set orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.yaw);
    transform_stamped.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_.sendTransform(transform_stamped);
}

void PepperArucoLocalizer::loadMarkerPositions() {
    // Markers on the LEFT wall (x ≈ 0, facing +X direction)
    known_markers_[35] = {cv::Vec3d(0.0, 2.4, 1.74), cv::Vec3d(0.0, 0.0, 0.0)};        // facing +X
    known_markers_[40] = {cv::Vec3d(0.0, 4.2, 1.72), cv::Vec3d(0.0, 0.0, 0.0)};        // facing +X  
    known_markers_[45] = {cv::Vec3d(0.0, 6.0, 1.74), cv::Vec3d(0.0, 0.0, 0.0)};        // facing +X
    known_markers_[50] = {cv::Vec3d(0.0, 7.2, 1.74), cv::Vec3d(0.0, 0.0, 0.0)};        // facing +X
    
    
    // Markers on the RIGHT wall (x ≈ 6.6-6.8, facing -X direction)  
    
    known_markers_[75] = {cv::Vec3d(6.8, 7.2, 1.72), cv::Vec3d(0.0, 0.0, M_PI)};       // facing -X
    known_markers_[80] = {cv::Vec3d(6.8, 5.1, 1.71), cv::Vec3d(0.0, 0.0, M_PI)};       // facing -X
    known_markers_[85] = {cv::Vec3d(6.8, 3.0, 1.67), cv::Vec3d(0.0, 0.0, M_PI)};       // facing -X
    
    
    // Markers on the BOTTOM wall (y ≈ 0-4, facing +Y direction)
    known_markers_[10] = {cv::Vec3d(6.6, 1.25, 1.64), cv::Vec3d(0.0, 0.0, M_PI/2)};      // facing +Y
    known_markers_[15] = {cv::Vec3d(3.2, 3.85, 0.61), cv::Vec3d(0.0, 0.0, M_PI/2)};    // facing +Y
    known_markers_[20] = {cv::Vec3d(3.8, 0.1, 1.40), cv::Vec3d(0.0, 0.0, M_PI/2)};  // facing +Y
    known_markers_[25] = {cv::Vec3d(2.0, 0.02, 1.74), cv::Vec3d(0.0, 0.0, M_PI/2)}; // facing +Y
    known_markers_[30] = {cv::Vec3d(2.0, 3.8, 0.61), cv::Vec3d(0.0, 0.0, M_PI/2)};     // facing +Y
    
    // Markers on the TOP wall (y ≈ 9.8-9.94, facing -Y direction)
    known_markers_[55] = {cv::Vec3d(0.1, 9.84, 1.72), cv::Vec3d(0.0, 0.0, -M_PI/2)};       // facing -Y (corner)
    known_markers_[60] = {cv::Vec3d(2.0, 9.94, 0.86), cv::Vec3d(0.0, 0.0, -M_PI/2)};   // facing -Y
    known_markers_[65] = {cv::Vec3d(5.0, 9.94, 0.86), cv::Vec3d(0.0, 0.0, -M_PI/2)};   // facing -Y
    known_markers_[70] = {cv::Vec3d(6.6, 9.84, 1.67), cv::Vec3d(0.0, 0.0, -M_PI/2)};      // facing -Y (corner)
    
    ROS_INFO("Loaded %zu marker positions with wall orientations", known_markers_.size());
}

void PepperArucoLocalizer::loadCameraParameters() {
    // Your Pepper robot's calibrated camera parameters
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_.at<double>(0,0) = 911.6033325195312;
    camera_matrix_.at<double>(0,2) = 655.0755615234375;
    camera_matrix_.at<double>(1,1) = 910.8851318359375;
    camera_matrix_.at<double>(1,2) = 363.9165954589844;
    camera_matrix_.at<double>(2,2) = 1.0;
    
    // No distortion coefficients
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    
    ROS_INFO("Camera parameters loaded (fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f)", 
             camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(1,1),
             camera_matrix_.at<double>(0,2), camera_matrix_.at<double>(1,2));
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pepper_aruco_localizer");
    
    PepperArucoLocalizer localizer;
    
    ros::spin();
    
    return 0;
}