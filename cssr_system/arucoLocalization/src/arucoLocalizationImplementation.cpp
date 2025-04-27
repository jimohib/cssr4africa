// #include <ros/ros.h>
// #include <aruco/aruco.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <cmath> // for std::isnan and std::isinf
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>

// class ArucoLocalization {
// public:
//     ArucoLocalization() : it_(nh_) {
//         // Subscribe to the RealSense camera image topic
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoLocalization::imageCallback, this);

//         // Load camera calibration parameters (replace with your RealSense camera intrinsics)
//         camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.5798950195312, 0.0, 633.3435668945312, 0.0, 910.5232543945312, 387.16180419921875, 0.0, 0.0, 1.0);
//         dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

//         // Initialize ArUco marker detector
//         detector_.setDictionary("ARUCO");
//     }

//     // Check for NaN or infinity in Rvec and Tvec
//     bool isValid(const cv::Mat& vec) {
//         for (int i = 0; i < vec.rows; i++) {
//             for (int j = 0; j < vec.cols; j++) {
//                 if (std::isnan(vec.at<double>(i, j)) || std::isinf(vec.at<double>(i, j))) {
//                     return false;
//                 }
//             }
//         }
//         return true;
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         ROS_INFO("Image received!");
//         try {
//             // Convert ROS image message to OpenCV image
//             cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
//             ROS_INFO("Image converted to OpenCV format.");

//             // Detect markers
//             std::vector<aruco::Marker> markers = detector_.detect(image);
//             ROS_INFO("Detected %zu markers.", markers.size());

//             // Process detected markers
//             for (auto& marker : markers) {
//                 marker.draw(image, cv::Scalar(0, 0, 255), 2);
//                 ROS_INFO("Processing marker ID: %d", marker.id);

//                  // Draw markers in red
//                  marker.draw(image, cv::Scalar(0, 0, 255), 2);

//                 // Check if marker data is valid
//                 if (marker.Rvec.empty() || marker.Tvec.empty()) {
//                     ROS_ERROR("Invalid marker data (Rvec or Tvec is empty).");
//                     continue;
//                 }

//                 // Check for NaN or infinity
//                 if (!isValid(marker.Rvec) || !isValid(marker.Tvec)) {
//                     ROS_ERROR("Invalid marker data (Rvec or Tvec contains NaN or infinity).");
//                     continue;
//                 }

//                 // Debug: Print Rvec and Tvec
//                 ROS_INFO("Rvec: %f, %f, %f", marker.Rvec.at<double>(0, 0), marker.Rvec.at<double>(1, 0), marker.Rvec.at<double>(2, 0));
//                 ROS_INFO("Tvec: %f, %f, %f", marker.Tvec.at<double>(0, 0), marker.Tvec.at<double>(1, 0), marker.Tvec.at<double>(2, 0));


//                 ROS_INFO("Marker size: %f", marker_size_);
//                 ROS_INFO("Camera matrix: %f, %f, %f, %f, %f, %f, %f, %f, %f",
//                         camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
//                         camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
//                         camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2));
//                 ROS_INFO("Distortion coefficients: %f, %f, %f, %f, %f",
//                         dist_coeffs_.at<double>(0, 0), dist_coeffs_.at<double>(0, 1), dist_coeffs_.at<double>(0, 2),
//                         dist_coeffs_.at<double>(0, 3), dist_coeffs_.at<double>(0, 4));
                
                
//                 // Calculate marker pose
//                 marker.calculateExtrinsics(marker_size_, camera_matrix_, dist_coeffs_);
//                 ROS_INFO("Marker %d detected at position: (%f, %f, %f)", marker.id, marker.Tvec.at<double>(0, 0), marker.Tvec.at<double>(1, 0), marker.Tvec.at<double>(2, 0));

                
//                 ROS_INFO("Rvec: %f, %f, %f", marker.Rvec.at<double>(0, 0), marker.Rvec.at<double>(1, 0), marker.Rvec.at<double>(2, 0));
//                 ROS_INFO("Tvec: %f, %f, %f", marker.Tvec.at<double>(0, 0), marker.Tvec.at<double>(1, 0), marker.Tvec.at<double>(2, 0));

                
//                 // Check if extrinsics calculation succeeded
//                 if (marker.Rvec.empty() || marker.Tvec.empty()) {
//                     ROS_ERROR("Failed to calculate extrinsics for marker ID: %d", marker.id);
//                     continue;
//                 }
                
//                 // Get translation and rotation vectors
//                 cv::Mat rvec = marker.Rvec;
//                 cv::Mat tvec = marker.Tvec;

//                 // Convert to TF transform
//                 tf::Transform transform;
//                 transform.setOrigin(tf::Vector3(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
//                 tf::Quaternion q;
//                 q.setRPY(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0));
//                 transform.setRotation(q);

//                 // Publish transform
//                 br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
//                 ROS_INFO("Published transform for marker ID: %d", marker.id);
//             }

//             cv::imshow("Detected Markers", image);
//             cv::waitKey(1);

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//         } catch (const std::exception& e) {
//             ROS_ERROR("Exception in imageCallback: %s", e.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     aruco::MarkerDetector detector_;
//     cv::Mat camera_matrix_;
//     cv::Mat dist_coeffs_;
//     tf::TransformBroadcaster br_;
//     double marker_size_ = 0.2; // Size of the ArUco marker in meters
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_localization");
//     ArucoLocalization aruco_localization;
//     ros::spin();
//     return 0;
// }










// #include <ros/ros.h>
// #include <aruco/aruco.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <nav_msgs/Odometry.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/aruco.hpp>


// class ArucoLocalization {
// public:
//     ArucoLocalization() : it_(nh_) {
//         // Subscribe to RealSense camera image topic
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoLocalization::imageCallback, this);
        
//         // Subscribe to Pepper's odometry topic
//         odom_sub_ = nh_.subscribe("/pepper_robot/odom", 10, &ArucoLocalization::odomCallback, this);

//         // Load camera calibration parameters
//         camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.58, 0.0, 633.34, 
//                                                      0.0, 910.52, 387.16, 
//                                                      0.0, 0.0, 1.0);
//         dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

//         // Initialize ArUco detector
//         detector_.setDictionary(cv::aruco::DICT_4X4_50);

//         // Initialize EKF
//         estimated_pose_.setIdentity();
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//         // Store Pepper's odometry pose
//         odom_x_ = msg->pose.pose.position.x;
//         odom_y_ = msg->pose.pose.position.y;
//         odom_theta_ = tf::getYaw(msg->pose.pose.orientation);
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             // Convert ROS image message to OpenCV
//             cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

//             // Detect markers
//             std::vector<aruco::Marker> markers = detector_.detect(image);

//             for (auto& marker : markers) {
//                 // Estimate marker pose
//                 marker.calculateExtrinsics(marker_size_, camera_matrix_, dist_coeffs_);

//                 // Convert marker pose to TF transform
//                 tf::Transform marker_transform;
//                 marker_transform.setOrigin(tf::Vector3(marker.Tvec.at<double>(0, 0), 
//                                                        marker.Tvec.at<double>(1, 0), 
//                                                        marker.Tvec.at<double>(2, 0)));
//                 tf::Quaternion q;
//                 q.setRPY(marker.Rvec.at<double>(0, 0), 
//                          marker.Rvec.at<double>(1, 0), 
//                          marker.Rvec.at<double>(2, 0));
//                 marker_transform.setRotation(q);


//                 // Combine with odometry using a simple Kalman filter update
//                 // estimated_pose_ = estimated_pose_.lerp(marker_transform, 0.5);


//                 // Perform linear interpolation manually
//                 tf::Vector3 interpolated_position = estimated_pose_.getOrigin().lerp(marker_transform.getOrigin(), 0.5);
//                 tf::Quaternion interpolated_rotation = estimated_pose_.getRotation().slerp(marker_transform.getRotation(), 0.5);

//                 // Update estimated_pose_
//                 estimated_pose_.setOrigin(interpolated_position);
//                 estimated_pose_.setRotation(interpolated_rotation);


//                 // Publish transform
//                 br_.sendTransform(tf::StampedTransform(estimated_pose_, ros::Time::now(), "odom", "base_footprint"));

//                 ROS_INFO("Published marker localization for marker ID: %d", marker.id);
//             }

//             cv::imshow("Aruco Detection", image);
//             cv::waitKey(1);

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     ros::Subscriber odom_sub_;
//     aruco::MarkerDetector detector_;
//     cv::Mat camera_matrix_;
//     cv::Mat dist_coeffs_;
//     tf::TransformBroadcaster br_;
//     double marker_size_ = 0.2; // Marker size in meters
//     tf::Transform estimated_pose_; // Estimated pose using ArUco + Odometry
//     double odom_x_, odom_y_, odom_theta_; // Odometry values
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_localization");
//     ArucoLocalization aruco_localization;
//     ros::spin();
//     return 0;
// }









// #include <ros/ros.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <nav_msgs/Odometry.h>

// class ArucoLocalization {
// public:
//     ArucoLocalization() : it_(nh_) {
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoLocalization::imageCallback, this);
//         odom_sub_ = nh_.subscribe("/pepper_robot/odom", 10, &ArucoLocalization::odomCallback, this);

//         // Load camera parameters
//         camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.58, 0.0, 633.34, 
//                                                      0.0, 910.52, 387.16, 
//                                                      0.0, 0.0, 1.0);
//         dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

//         // ✅ FIX: Correct way to initialize ArUco dictionary
//         dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//         parameters_ = cv::aruco::DetectorParameters::create();

//         estimated_pose_.setIdentity();
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//         odom_x_ = msg->pose.pose.position.x;
//         odom_y_ = msg->pose.pose.position.y;
//         odom_theta_ = tf::getYaw(msg->pose.pose.orientation);
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr;
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;

//             std::vector<int> marker_ids;
//             std::vector<std::vector<cv::Point2f>> marker_corners;
//             cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

//             if (marker_ids.empty()) {
//                 ROS_WARN("No markers detected.");
//                 return;
//             }

//             std::vector<cv::Vec3d> rvecs, tvecs;
//             cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

//             for (size_t i = 0; i < marker_ids.size(); i++) {
//                 cv::Mat rotation_matrix;
//                 cv::Rodrigues(rvecs[i], rotation_matrix);

//                 tf::Transform marker_transform;
//                 marker_transform.setOrigin(tf::Vector3(tvecs[i][0], tvecs[i][1], tvecs[i][2]));
//                 tf::Quaternion q;
//                 q.setRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]);
//                 marker_transform.setRotation(q);

//                 // ✅ FIX: Safe interpolation
//                 tf::Vector3 interpolated_position = estimated_pose_.getOrigin().lerp(marker_transform.getOrigin(), 0.5);
//                 tf::Quaternion interpolated_rotation = estimated_pose_.getRotation().slerp(marker_transform.getRotation(), 0.5);

//                 estimated_pose_.setOrigin(interpolated_position);
//                 estimated_pose_.setRotation(interpolated_rotation);

//                 br_.sendTransform(tf::StampedTransform(estimated_pose_, ros::Time::now(), "odom", "base_footprint"));
//                 ROS_INFO("Published marker localization for marker ID: %d", marker_ids[i]);
//             }

//             cv::imshow("Aruco Detection", image);
//             cv::waitKey(1);

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     ros::Subscriber odom_sub_;
//     cv::Ptr<cv::aruco::Dictionary> dictionary_;
//     cv::Ptr<cv::aruco::DetectorParameters> parameters_;
//     cv::Mat camera_matrix_, dist_coeffs_;
//     tf::TransformBroadcaster br_;
//     tf::Transform estimated_pose_;
//     double marker_size_ = 0.2;
//     double odom_x_, odom_y_, odom_theta_;
// };


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_localization");
//     ArucoLocalization aruco_localization;
//     ros::spin();
//     return 0;
// }










// #include <ros/ros.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <nav_msgs/Odometry.h>

// class ArucoLocalization {
// public:
//     ArucoLocalization() : it_(nh_) {
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoLocalization::imageCallback, this);
//         odom_sub_ = nh_.subscribe("/naoqi_driver/odom", 10, &ArucoLocalization::odomCallback, this);

//         // ✅ Load camera parameters (adjust according to your calibration)
//         camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.58, 0.0, 633.34, 
//                                                      0.0, 910.52, 387.16, 
//                                                      0.0, 0.0, 1.0);
//         dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

//         // ✅ Initialize ArUco dictionary
//         dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//         parameters_ = cv::aruco::DetectorParameters::create();

//         estimated_pose_.setIdentity();

//         // ✅ Initialize smoothing values
//         last_tvec_ = cv::Vec3d(0, 0, 0);
//         last_rvec_ = cv::Vec3d(0, 0, 0);
//         first_marker_detected_ = false;
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//         odom_x_ = msg->pose.pose.position.x;
//         odom_y_ = msg->pose.pose.position.y;
//         odom_theta_ = tf::getYaw(msg->pose.pose.orientation);
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         try {
//             cv_bridge::CvImagePtr cv_ptr;
//             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;

//             std::vector<int> marker_ids;
//             std::vector<std::vector<cv::Point2f>> marker_corners;
//             cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

//             if (marker_ids.empty()) {
//                 ROS_WARN_THROTTLE(2.0, "No markers detected."); // ✅ Reduce log spam
//                 return;
//             }

//             std::vector<cv::Vec3d> rvecs, tvecs;
//             cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

//             for (size_t i = 0; i < marker_ids.size(); i++) {
//                 cv::Mat rotation_matrix;
//                 cv::Rodrigues(rvecs[i], rotation_matrix);

//                 // ✅ Apply a Low-pass filter (smoothing factor 0.7)
//                 double alpha = 0.7;
//                 last_tvec_ = alpha * last_tvec_ + (1 - alpha) * tvecs[i];
//                 last_rvec_ = alpha * last_rvec_ + (1 - alpha) * rvecs[i];

//                 // ✅ Convert filtered values to TF transform
//                 tf::Transform marker_transform;
//                 marker_transform.setOrigin(tf::Vector3(last_tvec_[0], last_tvec_[1], last_tvec_[2]));
//                 tf::Quaternion q;
//                 q.setRPY(last_rvec_[0], last_rvec_[1], last_rvec_[2]);
//                 marker_transform.setRotation(q);

//                 // ✅ If first marker detected, set initial pose
//                 if (!first_marker_detected_) {
//                     estimated_pose_ = marker_transform;
//                     first_marker_detected_ = true;
//                 } else {
//                     // ✅ Safe interpolation to smooth localization jumps
//                     tf::Vector3 interpolated_position = estimated_pose_.getOrigin().lerp(marker_transform.getOrigin(), 0.5);
//                     tf::Quaternion interpolated_rotation = estimated_pose_.getRotation().slerp(marker_transform.getRotation(), 0.5);

//                     estimated_pose_.setOrigin(interpolated_position);
//                     estimated_pose_.setRotation(interpolated_rotation);
//                 }

//                 // ✅ Publish transform ONLY if change is significant
//                 if (shouldPublishUpdate(marker_transform)) {
//                     br_.sendTransform(tf::StampedTransform(estimated_pose_, ros::Time::now(), "odom", "base_footprint"));
//                     ROS_INFO("Published marker localization for marker ID: %d", marker_ids[i]);
//                 }
//             }

//             cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
//             cv::imshow("Aruco Detection", image);
//             cv::waitKey(1);

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

//     // ✅ Function to check if a significant change in localization occurred
//     bool shouldPublishUpdate(const tf::Transform& new_pose) {
//         double position_change = (new_pose.getOrigin() - estimated_pose_.getOrigin()).length();
//         double rotation_change = new_pose.getRotation().angle(estimated_pose_.getRotation());

//         const double POSITION_THRESHOLD = 0.05; // 5 cm
//         const double ROTATION_THRESHOLD = 0.05; // ~2.8 degrees

//         return (position_change > POSITION_THRESHOLD || rotation_change > ROTATION_THRESHOLD);
//     }

// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     ros::Subscriber odom_sub_;
//     cv::Ptr<cv::aruco::Dictionary> dictionary_;
//     cv::Ptr<cv::aruco::DetectorParameters> parameters_;
//     cv::Mat camera_matrix_, dist_coeffs_;
//     tf::TransformBroadcaster br_;
//     tf::Transform estimated_pose_;
//     cv::Vec3d last_tvec_, last_rvec_;
//     double marker_size_ = 0.2;
//     double odom_x_, odom_y_, odom_theta_;
//     bool first_marker_detected_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_localization");
//     ArucoLocalization aruco_localization;
//     ros::spin();
//     return 0;
// }










// #include <ros/ros.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/Image.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h>

// class ArucoLocalization {
// public:
//     ArucoLocalization() : it_(nh_) {
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoLocalization::imageCallback, this);
//         odom_sub_ = nh_.subscribe("/pepper_robot/odom", 10, &ArucoLocalization::odomCallback, this);
//         pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco_localization/pose", 10);

//         camera_matrix_ = (cv::Mat_<double>(3, 3) << 910.58, 0.0, 633.34, 
//                                                      0.0, 910.52, 387.16, 
//                                                      0.0, 0.0, 1.0);
//         dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

//         dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
//         parameters_ = cv::aruco::DetectorParameters::create();

//         last_update_time_ = ros::Time::now();
//         last_x_ = last_y_ = 0.0;  // Store last position to track movement
//         movement_threshold_ = 0.5; // Minimum distance to move before updating
//         update_interval_ = ros::Duration(5.0); // Update every 5 seconds
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//         odom_x_ = msg->pose.pose.position.x;
//         odom_y_ = msg->pose.pose.position.y;
//         odom_theta_ = tf::getYaw(msg->pose.pose.orientation);
//     }

//     void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//         ros::Time now = ros::Time::now();

//         // ✅ Check if the last update was too recent
//         if (now - last_update_time_ < update_interval_) {
//             ROS_INFO_THROTTLE(5, "Skipping update - time interval not reached.");
//             return;
//         }

//         try {
//             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//             cv::Mat image = cv_ptr->image;

//             std::vector<int> marker_ids;
//             std::vector<std::vector<cv::Point2f>> marker_corners;
//             cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids, parameters_);

//             if (marker_ids.empty()) {
//                 ROS_WARN_THROTTLE(5, "No markers detected.");
//                 return;
//             }

//             std::vector<cv::Vec3d> rvecs, tvecs;
//             cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

//             for (size_t i = 0; i < marker_ids.size(); i++) {
//                 double x = tvecs[i][0];
//                 double y = tvecs[i][1];

//                 // ✅ Check if the robot has moved significantly before updating
//                 double distance_moved = sqrt(pow(x - last_x_, 2) + pow(y - last_y_, 2));
//                 if (distance_moved < movement_threshold_) {
//                     ROS_INFO_THROTTLE(5, "Skipping update - robot hasn't moved significantly.");
//                     return;
//                 }

//                 // Update last known position
//                 last_x_ = x;
//                 last_y_ = y;
//                 last_update_time_ = now;

//                 // Publish updated pose
//                 geometry_msgs::PoseStamped pose_msg;
//                 pose_msg.header.stamp = now;
//                 pose_msg.header.frame_id = "map";
//                 pose_msg.pose.position.x = x;
//                 pose_msg.pose.position.y = y;
//                 pose_msg.pose.position.z = 0.0;
//                 tf::quaternionTFToMsg(tf::createQuaternionFromRPY(rvecs[i][0], rvecs[i][1], rvecs[i][2]), pose_msg.pose.orientation);

//                 pose_pub_.publish(pose_msg);
//                 ROS_INFO("Published absolute pose from marker ID: %d", marker_ids[i]);
//             }

//             cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
//             cv::imshow("Aruco Detection", image);
//             cv::waitKey(1);

//         } catch (cv_bridge::Exception& e) {
//             ROS_ERROR("cv_bridge exception: %s", e.what());
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     ros::Subscriber odom_sub_;
//     ros::Publisher pose_pub_;
//     cv::Ptr<cv::aruco::Dictionary> dictionary_;
//     cv::Ptr<cv::aruco::DetectorParameters> parameters_;
//     cv::Mat camera_matrix_, dist_coeffs_;
//     ros::Time last_update_time_;
//     double last_x_, last_y_;
//     double movement_threshold_;
//     ros::Duration update_interval_;
//     double odom_x_, odom_y_, odom_theta_;
//     double marker_size_ = 0.2;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "aruco_localization");
//     ArucoLocalization aruco_localization;
//     ros::spin();
//     return 0;
// }














#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>


class AbsoluteLocalization
{
public:
  AbsoluteLocalization(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), it_(nh_)
  {

    cv::namedWindow("Aruco Debug", cv::WINDOW_NORMAL);


    // --- Load parameters ---
    pnh.param("marker_dictionary", dict_id_, int(cv::aruco::DICT_4X4_100));
    pnh.param("marker_size",      marker_size_,      0.20);    // meters
    pnh.param("alpha",            alpha_,            0.2);     // complementary filter gain
    pnh.param("movement_threshold", movement_threshold_, 0.10); // meters
    pnh.param("rotation_threshold_deg", rotation_threshold_deg_, 5.0); // degrees
    pnh.param("update_interval",  update_interval_sec_, 2.0);    // seconds

    // --- Load known marker poses from param server ---
    // param “marker_positions” should be a list of dicts: [{id: 10, x: 1.0, y:2.0, yaw:90.0}, …]
    XmlRpc::XmlRpcValue mlist;
    if (!pnh.getParam("marker_positions", mlist) || mlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_FATAL("marker_positions parameter missing or not an array");
      ros::shutdown(); return;
    }
    for (int i = 0; i < mlist.size(); ++i) {
      auto &entry = mlist[i];
      int id        = int(entry["id"]);
      double mx     = double(entry["x"]);
      double my     = double(entry["y"]);
      double myaw   = double(entry["yaw"])*M_PI/180.0;  // convert to radians

      geometry_msgs::Pose pose;
      pose.position.x = mx;
      pose.position.y = my;
      pose.position.z = 0;
      tf::Quaternion q = tf::createQuaternionFromYaw(myaw);
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      marker_map_[id] = pose;
    }

    // --- ArUco setup ---
    dictionary_ = cv::aruco::getPredefinedDictionary(dict_id_);
    detector_params_ = cv::aruco::DetectorParameters::create();

    // --- ROS topics & services ---
    image_sub_      = it_.subscribe("image",        1, &AbsoluteLocalization::imageCb, this);
    caminfo_sub_    = nh_.subscribe("camera_info",  1, &AbsoluteLocalization::camInfoCb, this);
    odom_sub_       = nh_.subscribe("odom",         5, &AbsoluteLocalization::odomCb, this);
    pose_pub_       = nh_.advertise<geometry_msgs::PoseStamped>("absolute_pose", 10);

    got_cam_info_ = false;
    last_update_ = ros::Time(0);
  }

private:
  // --- Callbacks ---
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& ci) {
    if (got_cam_info_) return;
    // fill camera matrix
    cv::Mat K(3,3,CV_64F);
    for (int i=0;i<9;i++) K.at<double>(i/3,i%3) = ci->K[i];
    camera_matrix_ = K.clone();
    dist_coeffs_ = cv::Mat(ci->D.size(),1,CV_64F);
    for (size_t i=0;i<ci->D.size();i++)
      dist_coeffs_.at<double>(i,0) = ci->D[i];
    got_cam_info_ = true;
    ROS_INFO("Camera calibrated.");
  }

  void odomCb(const nav_msgs::OdometryConstPtr& odom) {
    odom_x_   = odom->pose.pose.position.x;
    odom_y_   = odom->pose.pose.position.y;
    odom_yaw_ = tf::getYaw(odom->pose.pose.orientation);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO(">> imageCb() fired. Got camera_info? %s", got_cam_info_ ? "yes":"no");
    if (!got_cam_info_) return;
    ros::Time now = ros::Time::now();
    if ((now - last_update_).toSec() < update_interval_sec_) return;

    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(img, dictionary_, corners, ids, detector_params_);
    ROS_INFO("Detected %zu ArUco markers.", ids.size());
    if (ids.empty()) return;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
      corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    
    // DRAW your markers & axes
    cv::aruco::drawDetectedMarkers(img, corners, ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      cv::aruco::drawAxis(img,
                          camera_matrix_, dist_coeffs_,
                          rvecs[i], tvecs[i],
                          marker_size_*0.5f);
    }
    cv::imshow("Aruco Debug", img);
    cv::waitKey(1);

    

    // fuse multiple marker measurements
    double sum_x=0, sum_y=0, sum_s=0;
    double sum_cos=0, sum_sin=0;
    for (size_t i=0;i<ids.size();i++) {
      auto it = marker_map_.find(ids[i]);
      if (it==marker_map_.end()) continue;
      // world→marker
      double mx = it->second.position.x;
      double my = it->second.position.y;
      double myaw = tf::getYaw(it->second.orientation);

      // marker→camera
      double Xc =  tvecs[i][0];
      double Yc =  tvecs[i][1];
      double c  = 1.0/std::hypot(Xc,Yc);   // weight = 1/dist

      // camera position in world
      double wx = mx +  cos(myaw)*Xc - sin(myaw)*Yc;
      double wy = my +  sin(myaw)*Xc + cos(myaw)*Yc;
      double yaw_cam = myaw + rvecs[i][2]; // in radians

      sum_x    += wx*c;
      sum_y    += wy*c;
      sum_s    += c;
      sum_cos  += cos(yaw_cam)*c;
      sum_sin  += sin(yaw_cam)*c;
    }
    if (sum_s == 0) return;
    double meas_x   = sum_x/sum_s;
    double meas_y   = sum_y/sum_s;
    double meas_yaw = atan2(sum_sin,sum_cos);


    ROS_INFO("Raw meas → x: %.2f, y: %.2f, yaw: %.1f°",
        meas_x, meas_y, meas_yaw*180.0/M_PI);



    // check movement threshold w.r.t. last published
    double dx = meas_x - last_x_;
    double dy = meas_y - last_y_;
    double dr = fabs(angles::normalize_angle(meas_yaw - last_yaw_)) * 180.0/M_PI;
    if (std::hypot(dx,dy) < movement_threshold_ &&
        dr            < rotation_threshold_deg_) {
      return;
    }

    // complementary fuse with odometry
    fused_x_   = alpha_*meas_x + (1-alpha_)*odom_x_;
    fused_y_   = alpha_*meas_y + (1-alpha_)*odom_y_;
    double dyaw = angles::shortest_angular_distance(odom_yaw_, meas_yaw);
    fused_yaw_ = angles::normalize_angle(odom_yaw_ + alpha_*dyaw);

    // publish
    geometry_msgs::PoseStamped out;
    out.header.stamp = now;
    out.header.frame_id = "map";
    out.pose.position.x = fused_x_;
    out.pose.position.y = fused_y_;
    out.pose.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromYaw(fused_yaw_);
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    pose_pub_.publish(out);

    // update for next threshold check
    last_update_ = now;
    last_x_      = meas_x;
    last_y_      = meas_y;
    last_yaw_    = meas_yaw;
  }

  // --- members ---
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber caminfo_sub_, odom_sub_;
  ros::Publisher  pose_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::Mat camera_matrix_, dist_coeffs_;
  bool got_cam_info_;

  std::map<int,geometry_msgs::Pose> marker_map_;

  // parameters
  int    dict_id_;
  double marker_size_, alpha_;
  double movement_threshold_, rotation_threshold_deg_;
  double update_interval_sec_;

  // state
  double odom_x_=0,   odom_y_=0,   odom_yaw_=0;
  double fused_x_=0,  fused_y_=0,  fused_yaw_=0;
  double last_x_=0,   last_y_=0,   last_yaw_=0;
  ros::Time last_update_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arucoLocalization");
  ros::NodeHandle nh, pnh("~");
  ROS_INFO(">>> Absolute localization node starting up, advertising /absolute_pose");
  AbsoluteLocalization node(nh, pnh);
  ROS_INFO(">>> Absolute localization started!");
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}
