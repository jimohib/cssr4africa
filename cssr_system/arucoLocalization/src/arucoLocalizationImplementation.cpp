#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <map>
#include <vector>
#include <string>
#include <iostream>

class ArucoLocalization {
private:
    // ROS related
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pose_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    
    // ArUco related
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    double marker_size_;
    
    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // Known marker locations and orientations
    std::map<int, cv::Point3d> marker_positions_;
    std::map<int, double> marker_orientations_;
    
    // Reference frame names
    std::string camera_frame_;
    std::string world_frame_;
    std::string robot_base_frame_;
    std::string camera_topic_;
    
    // Filtering parameters
    std::string filtering_method_;
    double filter_smoothing_factor_;
    
    // Last known pose for filtering
    geometry_msgs::Pose last_pose_;
    bool has_last_pose_ = false;

    // Debug publisher
    image_transport::Publisher debug_image_pub_;

public:
    ArucoLocalization() : 
        private_nh_("~"),
        it_(nh_),
        tfListener_(tfBuffer_)
    {
        // Load parameters
        loadParameters();
        
        // Set up publishers and subscribers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", 1);
        image_sub_ = it_.subscribe(camera_topic_, 1, &ArucoLocalization::imageCallback, this);
        debug_image_pub_ = it_.advertise("debug_image", 1);
        
        ROS_INFO("ArUco Localization node initialized");
    }
    
    void loadParameters() {
        // ArUco dictionary
        int dictionary_id;
        private_nh_.param<int>("aruco_dictionary_id", dictionary_id, 1); // Default to DICT_4X4_100
        
        // Select the dictionary based on the ID
        if (dictionary_id == 0) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        } else if (dictionary_id == 1) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        } else if (dictionary_id == 2) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        } else if (dictionary_id == 3) {
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
        } else {
            ROS_WARN("Unknown dictionary ID %d, defaulting to DICT_4X4_100", dictionary_id);
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        }
        
        parameters_ = cv::aruco::DetectorParameters::create();
        
        // Marker size
        private_nh_.param<double>("marker_size", marker_size_, 0.2);
        
        // Camera parameters
        std::vector<double> camera_matrix_vec, dist_coeffs_vec;
        private_nh_.getParam("camera_matrix", camera_matrix_vec);
        private_nh_.getParam("dist_coeffs", dist_coeffs_vec);
        
        if (camera_matrix_vec.size() == 9) {
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_vec.data()).clone();
        } else {
            ROS_ERROR("Invalid camera_matrix parameter. Using default values.");
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                           911.6033325195312, 0.0, 655.0755615234375,
                           0.0, 910.8851318359375, 363.9165954589844,
                           0.0, 0.0, 1.0);
        }
        
        if (!dist_coeffs_vec.empty()) {
            dist_coeffs_ = cv::Mat(dist_coeffs_vec.size(), 1, CV_64F, dist_coeffs_vec.data()).clone();
        } else {
            ROS_ERROR("Invalid dist_coeffs parameter. Using default values.");
            dist_coeffs_ = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        
        // Frame IDs
        private_nh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");
        private_nh_.param<std::string>("world_frame", world_frame_, "map");
        private_nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        private_nh_.param<std::string>("camera_topic", camera_topic_, "/camera/color/image_raw");
        
        // Filtering parameters
        private_nh_.param<std::string>("filtering_method", filtering_method_, "weighted");
        private_nh_.param<double>("filter_smoothing_factor", filter_smoothing_factor_, 0.3);
        
        // Load marker positions
        XmlRpc::XmlRpcValue markers;
        private_nh_.getParam("markers", markers);
        if (markers.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (XmlRpc::XmlRpcValue::iterator it = markers.begin(); it != markers.end(); ++it) {
                int marker_id = std::stoi(it->first);
                XmlRpc::XmlRpcValue marker_data = it->second;
                
                if (marker_data.getType() == XmlRpc::XmlRpcValue::TypeArray && marker_data.size() == 3) {
                    cv::Point3d point;
                    point.x = static_cast<double>(marker_data[0]);
                    point.y = static_cast<double>(marker_data[1]);
                    point.z = static_cast<double>(marker_data[2]);
                    
                    marker_positions_[marker_id] = point;
                    ROS_INFO_STREAM("Loaded marker " << marker_id << " at position (" 
                                  << point.x << ", " << point.y << ", " << point.z << ")");
                }
            }
        }
        
        // Load marker orientations
        XmlRpc::XmlRpcValue orientations;
        private_nh_.getParam("marker_orientations", orientations);
        if (orientations.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (XmlRpc::XmlRpcValue::iterator it = orientations.begin(); it != orientations.end(); ++it) {
                int marker_id = std::stoi(it->first);
                double orientation = static_cast<double>(it->second);
                marker_orientations_[marker_id] = orientation;
                ROS_INFO_STREAM("Loaded marker " << marker_id << " with orientation " << orientation << " degrees");
            }
        }
        
        if (marker_positions_.empty()) {
            ROS_WARN("No marker positions loaded. Localization will not work!");
        }
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, marker_corners, marker_ids, parameters_);
        
        // If at least one marker is detected
        if (!marker_ids.empty()) {
            // Draw markers for visualization
            cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
            
            // Estimate pose of each marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, 
                                               camera_matrix_, dist_coeffs_, rvecs, tvecs);
            
            // Draw axes for each detected marker
            for (size_t i = 0; i < rvecs.size(); ++i) {
                cv::drawFrameAxes(cv_ptr->image, camera_matrix_, dist_coeffs_, 
                                rvecs[i], tvecs[i], marker_size_ * 0.5);
                
                // Add marker ID text
                cv::Point2f center = (marker_corners[i][0] + marker_corners[i][2]) * 0.5f;
                cv::putText(cv_ptr->image, std::to_string(marker_ids[i]), 
                          center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            }
            
            // Estimate robot pose based on detected markers
            estimateRobotPose(marker_ids, rvecs, tvecs, msg->header.stamp);
        } else {
            ROS_WARN_THROTTLE(5.0, "No ArUco markers detected");
        }
        
        // Publish the processed image with markers drawn
        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", cv_ptr->image).toImageMsg();
        debug_image_pub_.publish(debug_msg);
    }
    
    void estimateRobotPose(const std::vector<int>& marker_ids, 
                         const std::vector<cv::Vec3d>& rvecs, 
                         const std::vector<cv::Vec3d>& tvecs,
                         const ros::Time& timestamp) {
        
        std::vector<geometry_msgs::Pose> marker_poses;
        std::vector<int> valid_marker_ids;
        
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            int id = marker_ids[i];
            
            // Skip if we don't know the world position of this marker
            if (marker_positions_.find(id) == marker_positions_.end()) {
                continue;
            }
            
            // Get marker position in world
            cv::Point3d marker_pos_world = marker_positions_[id];
            
            // Convert rotation vector to rotation matrix (camera to marker)
            cv::Mat rot_mat;
            cv::Rodrigues(rvecs[i], rot_mat);
            
            // Create transformation matrix from camera to marker
            cv::Mat transform_cam_to_marker = cv::Mat::eye(4, 4, CV_64F);
            rot_mat.copyTo(transform_cam_to_marker(cv::Rect(0, 0, 3, 3)));
            transform_cam_to_marker.at<double>(0, 3) = tvecs[i][0];
            transform_cam_to_marker.at<double>(1, 3) = tvecs[i][1];
            transform_cam_to_marker.at<double>(2, 3) = tvecs[i][2];
            
            // Invert to get marker to camera transformation
            cv::Mat transform_marker_to_cam = transform_cam_to_marker.inv();
            
            // Now account for marker orientation in world frame
            // Create rotation matrix for marker orientation in world frame
            cv::Mat rot_world_to_marker = cv::Mat::eye(3, 3, CV_64F);
            
            if (marker_orientations_.find(id) != marker_orientations_.end()) {
                double angle_rad = marker_orientations_[id] * CV_PI / 180.0;
                
                // Rotation around Z axis (most common for wall-mounted markers)
                rot_world_to_marker.at<double>(0, 0) = cos(angle_rad);
                rot_world_to_marker.at<double>(0, 1) = -sin(angle_rad);
                rot_world_to_marker.at<double>(1, 0) = sin(angle_rad);
                rot_world_to_marker.at<double>(1, 1) = cos(angle_rad);
            }
            
            // Create transformation matrix from world to marker
            cv::Mat transform_world_to_marker = cv::Mat::eye(4, 4, CV_64F);
            rot_world_to_marker.copyTo(transform_world_to_marker(cv::Rect(0, 0, 3, 3)));
            transform_world_to_marker.at<double>(0, 3) = marker_pos_world.x;
            transform_world_to_marker.at<double>(1, 3) = marker_pos_world.y;
            transform_world_to_marker.at<double>(2, 3) = marker_pos_world.z;
            
            // Calculate camera pose in world frame: world_T_camera = world_T_marker * marker_T_camera
            cv::Mat transform_world_to_cam = transform_world_to_marker * transform_marker_to_cam;
            
            // Extract position from transformation matrix
            cv::Mat t_vec = (cv::Mat_<double>(3, 1) << 
                          transform_world_to_cam.at<double>(0, 3),
                          transform_world_to_cam.at<double>(1, 3),
                          transform_world_to_cam.at<double>(2, 3));
            
            // Extract rotation matrix from transformation matrix
            cv::Mat r_mat = transform_world_to_cam(cv::Rect(0, 0, 3, 3));
            
            // Convert rotation matrix to quaternion
            tf2::Matrix3x3 tf_rot(
                r_mat.at<double>(0, 0), r_mat.at<double>(0, 1), r_mat.at<double>(0, 2),
                r_mat.at<double>(1, 0), r_mat.at<double>(1, 1), r_mat.at<double>(1, 2),
                r_mat.at<double>(2, 0), r_mat.at<double>(2, 1), r_mat.at<double>(2, 2)
            );
            
            tf2::Quaternion tf_quat;
            tf_rot.getRotation(tf_quat);
            
            // Create the pose
            geometry_msgs::Pose pose;
            
            // The position of the camera in the world frame
            pose.position.x = t_vec.at<double>(0, 0);
            pose.position.y = t_vec.at<double>(1, 0);
            pose.position.z = t_vec.at<double>(2, 0);
            
            // The orientation of the camera in the world frame
            pose.orientation.w = tf_quat.w();
            pose.orientation.x = tf_quat.x();
            pose.orientation.y = tf_quat.y();
            pose.orientation.z = tf_quat.z();
            
            marker_poses.push_back(pose);
            valid_marker_ids.push_back(id);
            
            ROS_INFO_THROTTLE(2.0, "Marker %d detected at distance %.2f meters", 
                            id, cv::norm(tvecs[i]));
        }
        
        if (marker_poses.empty()) {
            return;
        }
        
        // Apply the selected filtering method
        geometry_msgs::Pose camera_pose;
        if (filtering_method_ == "average") {
            camera_pose = averagePoses(marker_poses);
        } else if (filtering_method_ == "closest") {
            camera_pose = closestPose(marker_poses);
        } else if (filtering_method_ == "weighted") {
            camera_pose = weightedPoses(marker_poses, marker_ids);
        } else {
            camera_pose = marker_poses[0]; // Just use the first one
        }
        
        // Apply temporal filtering if we have a previous pose
        if (has_last_pose_) {
            camera_pose.position.x = filter_smoothing_factor_ * camera_pose.position.x + 
                                  (1 - filter_smoothing_factor_) * last_pose_.position.x;
            camera_pose.position.y = filter_smoothing_factor_ * camera_pose.position.y + 
                                  (1 - filter_smoothing_factor_) * last_pose_.position.y;
            camera_pose.position.z = filter_smoothing_factor_ * camera_pose.position.z + 
                                  (1 - filter_smoothing_factor_) * last_pose_.position.z;
            
            // SLERP for orientation
            tf2::Quaternion q1, q2, q_result;
            tf2::convert(last_pose_.orientation, q1);
            tf2::convert(camera_pose.orientation, q2);
            
            // Ensure quaternions are on the same hemisphere
            if (q1.dot(q2) < 0) {
                q2 = q2 * -1.0;
            }
            
            q_result = q1.slerp(q2, filter_smoothing_factor_);
            camera_pose.orientation = tf2::toMsg(q_result);
        }
        
        // Save for next iteration
        last_pose_ = camera_pose;
        has_last_pose_ = true;
        
        // Now get the transformation from camera to robot base
        try {
            geometry_msgs::TransformStamped camera_to_base = 
                tfBuffer_.lookupTransform(robot_base_frame_, camera_frame_, ros::Time(0));
            
            // Create and publish the TF from world to camera
            geometry_msgs::TransformStamped world_to_camera;
            world_to_camera.header.stamp = timestamp;
            world_to_camera.header.frame_id = world_frame_;
            world_to_camera.child_frame_id = camera_frame_;
            
            world_to_camera.transform.translation.x = camera_pose.position.x;
            world_to_camera.transform.translation.y = camera_pose.position.y;
            world_to_camera.transform.translation.z = camera_pose.position.z;
            world_to_camera.transform.rotation = camera_pose.orientation;
            
            tf_broadcaster_.sendTransform(world_to_camera);
            
            // Convert camera pose to robot base pose
            geometry_msgs::PoseWithCovarianceStamped robot_pose_msg;
            robot_pose_msg.header.stamp = timestamp;
            robot_pose_msg.header.frame_id = world_frame_;
            
            // Transform from camera to base link
            tf2::Transform tf_world_to_camera;
            tf2::Transform tf_camera_to_base;
            tf2::Transform tf_world_to_base;
            
            tf2::fromMsg(world_to_camera.transform, tf_world_to_camera);
            tf2::fromMsg(camera_to_base.transform, tf_camera_to_base);
            
            tf_world_to_base = tf_world_to_camera * tf_camera_to_base;
            
            geometry_msgs::Transform world_to_base_transform = tf2::toMsg(tf_world_to_base);
            
            robot_pose_msg.pose.pose.position.x = world_to_base_transform.translation.x;
            robot_pose_msg.pose.pose.position.y = world_to_base_transform.translation.y;
            robot_pose_msg.pose.pose.position.z = world_to_base_transform.translation.z;
            robot_pose_msg.pose.pose.orientation = world_to_base_transform.rotation;
            
            // Set a diagonal covariance based on distance to markers and number of markers
            double avg_distance = 0.0;
            for (size_t i = 0; i < marker_poses.size(); ++i) {
                double dx = marker_poses[i].position.x - camera_pose.position.x;
                double dy = marker_poses[i].position.y - camera_pose.position.y;
                double dz = marker_poses[i].position.z - camera_pose.position.z;
                avg_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            avg_distance /= marker_poses.size();
            
            // Base covariance on distance and number of markers
            // Higher confidence (lower covariance) with more markers and closer distances
            double pos_covariance = 0.01 + 0.02 * avg_distance / marker_poses.size();
            double rot_covariance = 0.01 + 0.02 * avg_distance / marker_poses.size();
            
            // Fill the covariance matrix (6x6 for x, y, z, roll, pitch, yaw)
            for (int i = 0; i < 36; ++i) {
                robot_pose_msg.pose.covariance[i] = 0.0;
            }
            robot_pose_msg.pose.covariance[0] = pos_covariance;  // x
            robot_pose_msg.pose.covariance[7] = pos_covariance;  // y
            robot_pose_msg.pose.covariance[14] = pos_covariance; // z
            robot_pose_msg.pose.covariance[21] = rot_covariance; // roll
            robot_pose_msg.pose.covariance[28] = rot_covariance; // pitch
            robot_pose_msg.pose.covariance[35] = rot_covariance; // yaw
            
            // Publish the robot pose
            pose_pub_.publish(robot_pose_msg);
            
            ROS_INFO_THROTTLE(1.0, "Robot pose: x=%.2f, y=%.2f, z=%.2f using %zu markers", 
                            robot_pose_msg.pose.pose.position.x,
                            robot_pose_msg.pose.pose.position.y,
                            robot_pose_msg.pose.pose.position.z,
                            marker_poses.size());
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "Transform from %s to %s not available: %s", 
                           camera_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        }
    }
    
    // Helper Methods for Pose Filtering
    
    geometry_msgs::Pose averagePoses(const std::vector<geometry_msgs::Pose>& poses) {
        geometry_msgs::Pose avg_pose;
        avg_pose.position.x = 0;
        avg_pose.position.y = 0;
        avg_pose.position.z = 0;
        
        tf2::Quaternion avg_quat(0, 0, 0, 0);
        
        for (const auto& pose : poses) {
            avg_pose.position.x += pose.position.x;
            avg_pose.position.y += pose.position.y;
            avg_pose.position.z += pose.position.z;
            
            tf2::Quaternion q;
            tf2::convert(pose.orientation, q);
            
            // Handle quaternion sign for proper averaging
            if (avg_quat.dot(q) < 0) {
                q = q * -1.0;
            }
            
            avg_quat = avg_quat + q;
        }
        
        // Compute average position
        avg_pose.position.x /= poses.size();
        avg_pose.position.y /= poses.size();
        avg_pose.position.z /= poses.size();
        
        // Normalize the quaternion
        avg_quat.normalize();
        avg_pose.orientation = tf2::toMsg(avg_quat);
        
        return avg_pose;
    }
    
    geometry_msgs::Pose closestPose(const std::vector<geometry_msgs::Pose>& poses) {
      // Simply return the pose of the closest marker (assuming closer is more accurate)
      double min_distance = std::numeric_limits<double>::max();
      size_t closest_idx = 0;
      
      for (size_t i = 0; i < poses.size(); ++i) {
          const auto& pose = poses[i];
          double distance = std::sqrt(pose.position.x * pose.position.x + 
                                   pose.position.y * pose.position.y + 
                                   pose.position.z * pose.position.z);
          
          if (distance < min_distance) {
              min_distance = distance;
              closest_idx = i;
          }
      }
      
      return poses[closest_idx];
  }
  
  geometry_msgs::Pose weightedPoses(const std::vector<geometry_msgs::Pose>& poses, 
                                  const std::vector<int>& marker_ids) {
      // Calculate weights based on the inverse of the distance
      std::vector<double> weights;
      double total_weight = 0.0;
      
      for (size_t i = 0; i < poses.size(); ++i) {
          const auto& pose = poses[i];
          double distance = std::sqrt(pose.position.x * pose.position.x + 
                                   pose.position.y * pose.position.y + 
                                   pose.position.z * pose.position.z);
          
          // Avoid division by zero and give higher weight to closer markers
          double weight = 1.0 / (distance + 0.1);
          weights.push_back(weight);
          total_weight += weight;
      }
      
      // Normalize weights
      for (auto& w : weights) {
          w /= total_weight;
      }
      
      // Calculate weighted average
      geometry_msgs::Pose avg_pose;
      avg_pose.position.x = 0;
      avg_pose.position.y = 0;
      avg_pose.position.z = 0;
      
      // For orientation averaging
      std::vector<tf2::Quaternion> quaternions;
      for (const auto& pose : poses) {
          tf2::Quaternion q;
          tf2::convert(pose.orientation, q);
          quaternions.push_back(q);
      }
      
      // Compute weighted average
      tf2::Quaternion avg_quat(0, 0, 0, 0);
      for (size_t i = 0; i < poses.size(); ++i) {
          avg_pose.position.x += weights[i] * poses[i].position.x;
          avg_pose.position.y += weights[i] * poses[i].position.y;
          avg_pose.position.z += weights[i] * poses[i].position.z;
          
          // For quaternions, ensure they're in the same hemisphere
          if (i > 0 && quaternions[0].dot(quaternions[i]) < 0) {
              quaternions[i] = quaternions[i] * -1.0;
          }
          
          tf2::Quaternion weighted_quat = quaternions[i] * weights[i];
          avg_quat = avg_quat + weighted_quat;
      }
      
      // Normalize the resulting quaternion
      avg_quat.normalize();
      avg_pose.orientation = tf2::toMsg(avg_quat);
      
      return avg_pose;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_localization");
  ArucoLocalization aruco_localization;
  ros::spin();
  return 0;
}