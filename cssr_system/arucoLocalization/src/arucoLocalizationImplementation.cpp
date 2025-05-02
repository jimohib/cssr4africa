/**
 * robot_localization_node.cpp
 * 
 * ROS node for robot localization using triangulation with landmarks.
 * Combines relative pose estimation (odometry and IMU) with absolute
 * pose estimation (triangulation with visual landmarks).
 * 
 * This implementation uses ArUco markers as landmarks and includes
 * ArUco detection as the primary method with fallback to basic feature detection.
 */

 #include <ros/ros.h>
 #include <sensor_msgs/Image.h>
 #include <sensor_msgs/JointState.h>
 #include <sensor_msgs/Imu.h>
 #include <nav_msgs/Odometry.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <std_srvs/Empty.h>
 #include <opencv2/opencv.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <yaml-cpp/yaml.h>
 #include <fstream>
 #include <vector>
 #include <string>
 #include <mutex>
 #include <opencv2/features2d.hpp>
 #include <opencv2/dnn.hpp>
 #include <opencv2/aruco.hpp>
 #include <cmath>
 
 /**
  * @brief Robot Localization Node
  * 
  * This ROS node determines the pose (position and orientation) of a robot in a Cartesian 
  * world frame of reference, using a combination of relative pose estimation (odometry and IMU)
  * and absolute pose estimation (triangulation with landmarks).
  */
 class RobotLocalizationNode {
 private:
     // ROS node handle
     ros::NodeHandle nh_;
     ros::NodeHandle private_nh_;
     
     // ROS subscribers
     ros::Subscriber rgb_camera_sub_;
     ros::Subscriber depth_camera_sub_;
     ros::Subscriber odom_sub_;
     ros::Subscriber imu_sub_;
     ros::Subscriber joint_states_sub_;
     
     // ROS publishers
     ros::Publisher pose_pub_;
     ros::Publisher image_pub_;
     
     // ROS service
     ros::ServiceServer reset_pose_service_;
     
     // Current pose data
     double x_, y_, theta_;
     std::mutex pose_mutex_;
     
     // Last update time
     ros::Time last_odom_time_;
     
     // Landmark information
     struct Landmark {
         int id;
         double x, y, theta;  // Position and orientation in world coordinates
     };
     std::vector<Landmark> landmarks_;
     
     // Parameters
     std::string config_file_;
     std::string topics_file_;
     std::string camera_params_file_;
     bool verbose_mode_;
     
     // Camera parameters
     cv::Mat camera_matrix_;
     cv::Mat dist_coeffs_;
     double marker_size_;
     
     // OpenCV and detection-related
     cv::Ptr<cv::Feature2D> feature_detector_;
     cv::dnn::Net yolo_net_;
     cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
     cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
     
     // IMU bias correction
     double gyro_bias_z_;
     
     // Head yaw angle
     double head_yaw_angle_;
     
     // Topic names
     std::string front_camera_topic_;
     std::string stereo_camera_topic_;
     std::string rgb_realsense_topic_;
     std::string depth_realsense_topic_;
     std::string odometry_topic_;
     std::string imu_topic_;
     std::string head_yaw_topic_;
     
     // Latest RGB image
     cv::Mat current_rgb_image_;
     cv::Mat current_depth_image_;
     std::mutex image_mutex_;
     
     // Flag to force absolute pose estimation
     bool force_absolute_pose_;
     
     /**
      * @brief Determine the points where 2 circles in a common plane intersect.
      * 
      * @param x0 x-coordinate of first circle center
      * @param y0 y-coordinate of first circle center
      * @param r0 radius of first circle
      * @param x1 x-coordinate of second circle center
      * @param y1 y-coordinate of second circle center
      * @param r1 radius of second circle
      * @param xi pointer to x-coordinate of first intersection point
      * @param yi pointer to y-coordinate of first intersection point
      * @param xi_prime pointer to x-coordinate of second intersection point
      * @param yi_prime pointer to y-coordinate of second intersection point
      * @return int 1 if circles intersect, 0 if they don't
      */
     int circle_circle_intersection(double x0, double y0, double r0,
                                   double x1, double y1, double r1,
                                   double *xi, double *yi,
                                   double *xi_prime, double *yi_prime) {
         double a, dx, dy, d, h, rx, ry;
         double x2, y2;
         
         // dx and dy are the vertical and horizontal distances between
         // the circle centers.
         dx = x1 - x0;
         dy = y1 - y0;
         
         // Determine the straight-line distance between the centers.
         d = hypot(dx, dy);
         
         // Check for solvability.
         if (d > (r0 + r1))
         {
             // No solution. circles do not intersect.
             return 0;
         }
         if (d < fabs(r0 - r1))
         {
             // No solution. one circle is contained in the other
             return 0;
         }
         
         // 'point 2' is the point where the line through the circle
         // intersection points crosses the line between the circle
         // centers.  
         
         // Determine the distance from point 0 to point 2.
         a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;
         
         // Determine the coordinates of point 2.
         x2 = x0 + (dx * a/d);
         y2 = y0 + (dy * a/d);
         
         // Determine the distance from point 2 to either of the
         // intersection points.
         h = sqrt((r0*r0) - (a*a));
         
         // Now determine the offsets of the intersection points from
         // point 2.
         rx = -dy * (h/d);
         ry = dx * (h/d);
         
         // Determine the absolute intersection points.
         *xi = x2 + rx;
         *xi_prime = x2 - rx;
         *yi = y2 + ry;
         *yi_prime = y2 - ry;
         
         return 1;
     }
     
     /**
      * @brief Determine the centre and radius of a circle given a chord and subtended angle
      * 
      * Since there are two circles through two points subtending a given angle, this
      * function returns the centres of both circles.
      * The circle centres are given in order of increasing distance from the origin.
      * 
      * @param x1 x-coordinate of first endpoint of chord
      * @param y1 y-coordinate of first endpoint of chord
      * @param x2 x-coordinate of second endpoint of chord
      * @param y2 y-coordinate of second endpoint of chord
      * @param alpha angle in degrees subtended by the chord
      * @param xc1 pointer to x-coordinate of first circle center
      * @param yc1 pointer to y-coordinate of first circle center
      * @param xc2 pointer to x-coordinate of second circle center
      * @param yc2 pointer to y-coordinate of second circle center
      * @param r pointer to radius of the circles
      * @return int 1 if successful
      */
     int circle_centre(double x1, double y1, double x2, double y2, double alpha, 
                     double *xc1, double *yc1, double *xc2, double *yc2, double *r) {
         const double PI = 3.14159265358979323846;
         
         double d;
         double h;
         double theta;
         double beta;
         double delta_x;
         double delta_y;
         double alphar;
         double temp_x;
         double temp_y;
         
         // Convert angle to radians
         alphar = PI * (alpha / 180.0);
         
         // Calculate distance between points
         d = sqrt((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
         
         // Handle special cases
         if (alpha == 0 || alpha == 180) {
             h = 0;
             *r = 0;
         }
         else {
             h = (d / 2) / tan(alphar);
             *r = (d / 2) / sin(alphar);
         }
         
         // Calculate direction of the chord
         theta = atan2(y2-y1, x2-x1);
         // Calculate direction perpendicular to the chord
         beta = theta - (PI / 2);
         
         // Calculate offsets to circle centers
         delta_x = h * cos(beta);
         delta_y = h * sin(beta);
         
         // Calculate centers of both possible circles
         *xc1 = (x1 + x2)/2 - delta_x;  
         *yc1 = (y1 + y2)/2 - delta_y;  
         
         *xc2 = (x1 + x2)/2 + delta_x; 
         *yc2 = (y1 + x2)/2 + delta_y; 
         
         // Sort them in order of increasing distance from the origin
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
 
 public:
     RobotLocalizationNode() : private_nh_("~") {
         // Load parameters
         private_nh_.param<std::string>("config_file", config_file_, "markers.yaml");
         private_nh_.param<std::string>("topics_file", topics_file_, "topics.dat");
         private_nh_.param<std::string>("camera_params_file", camera_params_file_, "camera_calibration.yaml");
         private_nh_.param<bool>("verbose", verbose_mode_, false);
         
         // Initialize pose
         x_ = 0.0;
         y_ = 0.0;
         theta_ = 0.0;
         
         // Initialize IMU bias
         gyro_bias_z_ = 0.0;
         
         // Initialize head yaw
         head_yaw_angle_ = 0.0;
         
         // Initialize absolute pose flag
         force_absolute_pose_ = false;
         
         // Load camera parameters from file
         loadCameraParameters();
         
         // Load landmarks configuration
         loadLandmarksConfig();
         
         // Setup subscribers and publishers based on topics file
         loadTopicsConfig();
         
         // Initialize feature detector (using ORB instead of SIFT)
         feature_detector_ = cv::ORB::create();
         
         // Initialize YOLO network
         initializeYOLO();
         
         // Initialize ArUco detector
         aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
         aruco_params_ = cv::aruco::DetectorParameters::create();
         
         // Create service
         reset_pose_service_ = nh_.advertiseService("robotLocalization/reset_pose", 
                                                   &RobotLocalizationNode::resetPoseCallback, this);
         
         // Setup publishers
         pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robotLocalization/pose", 10);
         image_pub_ = nh_.advertise<sensor_msgs::Image>("robotLocalization/landmark_detection", 1);
         
         ROS_INFO("Robot Localization Node initialized");
     }
     
     ~RobotLocalizationNode() {
         if (verbose_mode_) {
             cv::destroyAllWindows();
         }
     }
     
     void loadCameraParameters() {
         try {
             YAML::Node config = YAML::LoadFile(camera_params_file_);
             
             // Load camera matrix
             if (config["camera_matrix"]) {
                 std::vector<double> matrix_values = config["camera_matrix"].as<std::vector<double>>();
                 if (matrix_values.size() == 9) {
                     camera_matrix_ = cv::Mat(3, 3, CV_64F);
                     for (int i = 0; i < 3; i++) {
                         for (int j = 0; j < 3; j++) {
                             camera_matrix_.at<double>(i, j) = matrix_values[i * 3 + j];
                         }
                     }
                     ROS_INFO("Camera matrix loaded from file");
                 } else {
                     ROS_ERROR("Invalid camera matrix in file (expected 9 values)");
                     // Use default values as fallback
                     camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                         911.6033325195312, 0.0, 655.0755615234375,
                         0.0, 910.8851318359375, 363.9165954589844,
                         0.0, 0.0, 1.0);
                 }
             } else {
                 ROS_WARN("No camera_matrix in calibration file, using defaults");
                 camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                     911.6033325195312, 0.0, 655.0755615234375,
                     0.0, 910.8851318359375, 363.9165954589844,
                     0.0, 0.0, 1.0);
             }
             
             // Load distortion coefficients
             if (config["dist_coeffs"]) {
                 std::vector<double> dist_values = config["dist_coeffs"].as<std::vector<double>>();
                 if (dist_values.size() == 5) {
                     dist_coeffs_ = cv::Mat(1, 5, CV_64F);
                     for (int i = 0; i < 5; i++) {
                         dist_coeffs_.at<double>(0, i) = dist_values[i];
                     }
                     ROS_INFO("Distortion coefficients loaded from file");
                 } else {
                     ROS_ERROR("Invalid distortion coefficients in file (expected 5 values)");
                     // Use default values as fallback
                     dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
                 }
             } else {
                 ROS_WARN("No dist_coeffs in calibration file, using defaults");
                 dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
             }
             
             // Load marker size
             if (config["marker_size"]) {
                 marker_size_ = config["marker_size"].as<double>();
                 ROS_INFO("Marker size loaded from file: %f meters", marker_size_);
             } else {
                 ROS_WARN("No marker_size in calibration file, using default value of 0.2m");
                 marker_size_ = 0.2; // Default marker size in meters
             }
             
         } catch (const YAML::Exception& e) {
             ROS_ERROR("Error loading camera parameters: %s", e.what());
             ROS_WARN("Using default camera calibration parameters");
             
             // Default values as fallback
             camera_matrix_ = (cv::Mat_<double>(3, 3) << 
                 911.6033325195312, 0.0, 655.0755615234375,
                 0.0, 910.8851318359375, 363.9165954589844,
                 0.0, 0.0, 1.0);
                 
             dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
             marker_size_ = 0.2;
         }
     }
     
     void loadLandmarksConfig() {
         try {
             YAML::Node config = YAML::LoadFile(config_file_);
             
             if (config["marker_positions"]) {
                 auto markers = config["marker_positions"];
                 for (const auto& marker : markers) {
                     Landmark l;
                     l.id = marker["id"].as<int>();
                     l.x = marker["x"].as<double>();
                     l.y = marker["y"].as<double>();
                     l.theta = marker["theta"].as<double>() * M_PI / 180.0; // Convert to radians
                     landmarks_.push_back(l);
                     
                     ROS_INFO("Loaded marker ID: %d, Position: (%f, %f), Orientation: %f degrees", 
                             l.id, l.x, l.y, marker["theta"].as<double>());
                 }
             } else {
                 ROS_ERROR("No marker_positions defined in configuration file");
             }
         } catch (const YAML::Exception& e) {
             ROS_ERROR("Error loading landmarks configuration: %s", e.what());
         }
     }
     
     void loadTopicsConfig() {
         std::ifstream topics_file(topics_file_);
         if (!topics_file.is_open()) {
             ROS_ERROR("Could not open topics file: %s", topics_file_.c_str());
             return;
         }
         
         std::string line;
         std::map<std::string, std::string> topic_map;
         
         while (std::getline(topics_file, line)) {
             size_t delimiter_pos = line.find('=');
             if (delimiter_pos != std::string::npos) {
                 std::string key = line.substr(0, delimiter_pos);
                 std::string value = line.substr(delimiter_pos + 1);
                 
                 // Trim whitespace
                 key.erase(0, key.find_first_not_of(" \t"));
                 key.erase(key.find_last_not_of(" \t") + 1);
                 value.erase(0, value.find_first_not_of(" \t"));
                 value.erase(value.find_last_not_of(" \t") + 1);
                 
                 topic_map[key] = value;
             }
         }
         
         // Store topic names
         front_camera_topic_ = topic_map["FrontCamera"];
         stereo_camera_topic_ = topic_map["StereoCamera"];
         rgb_realsense_topic_ = topic_map["RGBRealSense"];
         depth_realsense_topic_ = topic_map["DepthRealSense"];
         odometry_topic_ = topic_map["Odometry"];
         imu_topic_ = topic_map["IMU"];
         head_yaw_topic_ = topic_map["HeadYaw"];
         
         // Set up subscribers
         if (!rgb_realsense_topic_.empty()) {
             rgb_camera_sub_ = nh_.subscribe(rgb_realsense_topic_, 1, 
                                            &RobotLocalizationNode::rgbCameraCallback, this);
             ROS_INFO("Subscribed to RGB RealSense topic: %s", rgb_realsense_topic_.c_str());
         }
         
         if (!depth_realsense_topic_.empty()) {
             depth_camera_sub_ = nh_.subscribe(depth_realsense_topic_, 1, 
                                              &RobotLocalizationNode::depthCameraCallback, this);
             ROS_INFO("Subscribed to Depth RealSense topic: %s", depth_realsense_topic_.c_str());
         }
         
         if (!odometry_topic_.empty()) {
             odom_sub_ = nh_.subscribe(odometry_topic_, 10, 
                                      &RobotLocalizationNode::odometryCallback, this);
             ROS_INFO("Subscribed to odometry topic: %s", odometry_topic_.c_str());
         }
         
         if (!imu_topic_.empty()) {
             imu_sub_ = nh_.subscribe(imu_topic_, 10, 
                                     &RobotLocalizationNode::imuCallback, this);
             ROS_INFO("Subscribed to IMU topic: %s", imu_topic_.c_str());
         }
         
         if (!head_yaw_topic_.empty()) {
             joint_states_sub_ = nh_.subscribe(head_yaw_topic_, 10, 
                                              &RobotLocalizationNode::jointStatesCallback, this);
             ROS_INFO("Subscribed to joint states topic: %s", head_yaw_topic_.c_str());
         }
     }
     
     void initializeYOLO() {
         try {
             // Load YOLO network - paths would need to be adjusted based on your setup
             yolo_net_ = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");
             yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
             yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
             ROS_INFO("YOLO network initialized");
         } catch (const cv::Exception& e) {
             ROS_WARN("Failed to initialize YOLO: %s", e.what());
             ROS_WARN("YOLO-based detection will not be available");
         }
     }
     
     void rgbCameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
         try {
             // Convert ROS image to OpenCV format
             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
             
             {
                 std::lock_guard<std::mutex> lock(image_mutex_);
                 current_rgb_image_ = cv_ptr->image.clone();
             }
             
             // Process the image for landmark detection
             processImage();
             
         } catch (cv_bridge::Exception& e) {
             ROS_ERROR("cv_bridge exception: %s", e.what());
         }
     }
     
     void depthCameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
         try {
             // Convert ROS image to OpenCV format
             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
             
             {
                 std::lock_guard<std::mutex> lock(image_mutex_);
                 current_depth_image_ = cv_ptr->image.clone();
             }
             
         } catch (cv_bridge::Exception& e) {
             ROS_ERROR("cv_bridge exception: %s", e.what());
         }
     }
     
     void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
         std::lock_guard<std::mutex> lock(pose_mutex_);
         
         // First odometry message - initialize time
         if (last_odom_time_.isZero()) {
             last_odom_time_ = msg->header.stamp;
             return;
         }
         
         // Calculate time delta
         double dt = (msg->header.stamp - last_odom_time_).toSec();
         last_odom_time_ = msg->header.stamp;
         
         if (dt <= 0) return;
         
         // Extract linear and angular velocities
         double vx = msg->twist.twist.linear.x;
         double vy = msg->twist.twist.linear.y;
         double vtheta = msg->twist.twist.angular.z;
         
         // Integrate motion
         double delta_x = (vx * cos(theta_) - vy * sin(theta_)) * dt;
         double delta_y = (vx * sin(theta_) + vy * cos(theta_)) * dt;
         double delta_theta = vtheta * dt;
         
         // Update pose
         x_ += delta_x;
         y_ += delta_y;
         theta_ += delta_theta;
         
         // Normalize theta to [-pi, pi]
         while (theta_ > M_PI) theta_ -= 2 * M_PI;
         while (theta_ < -M_PI) theta_ += 2 * M_PI;
         
         // Publish updated pose
         publishPose();
         
         if (verbose_mode_) {
             ROS_INFO("Odometry update - Position: (%f, %f), Orientation: %f", x_, y_, theta_);
         }
     }
     
     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
         // IMU processing can be used to complement odometry
         // This is a simple implementation that can be expanded
         
         // For initial bias estimation - can be improved with proper calibration
         static int bias_count = 0;
         static double bias_sum = 0.0;
         
         if (bias_count < 100) {
             bias_sum += msg->angular_velocity.z;
             bias_count++;
             
             if (bias_count == 100) {
                 gyro_bias_z_ = bias_sum / 100.0;
                 ROS_INFO("IMU gyro bias estimated: %f", gyro_bias_z_);
             }
             return;
         }
         
         // Use IMU data to improve orientation estimate
         // This could be expanded with a proper sensor fusion algorithm (e.g., EKF)
         double gyro_z = msg->angular_velocity.z - gyro_bias_z_;
         
         // For now, we just log the data in verbose mode
         if (verbose_mode_) {
             ROS_INFO("IMU angular velocity (bias-corrected): %f", gyro_z);
         }
     }
     
     void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
         // Find the HeadYaw joint in the message
         for (size_t i = 0; i < msg->name.size(); ++i) {
             if (msg->name[i] == "HeadYaw") {
                 head_yaw_angle_ = msg->position[i];
                 
                 if (verbose_mode_) {
                     ROS_INFO("Head yaw angle updated: %f", head_yaw_angle_);
                 }
                 break;
             }
         }
     }
     
     void processImage() {
         cv::Mat rgb_image;
         
         {
             std::lock_guard<std::mutex> lock(image_mutex_);
             if (current_rgb_image_.empty()) {
                 return;
             }
             rgb_image = current_rgb_image_.clone();
         }
         
         // Create a copy for visualization
         cv::Mat display_frame = rgb_image.clone();
         
         // Detect ArUco markers
         std::vector<int> detected_ids;
         std::vector<std::vector<cv::Point2f>> marker_corners;
         std::vector<cv::Vec3d> rvecs, tvecs;
         
         cv::aruco::detectMarkers(rgb_image, aruco_dict_, marker_corners, detected_ids, aruco_params_);
         
         // Draw detected markers
         if (!detected_ids.empty()) {
             cv::aruco::drawDetectedMarkers(display_frame, marker_corners, detected_ids);
             
             // Estimate pose for each marker
             cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_size_, 
                                                camera_matrix_, dist_coeffs_, 
                                                rvecs, tvecs);
             
             // Draw axes for each detected marker
             for (size_t i = 0; i < rvecs.size(); ++i) {
                 cv::aruco::drawAxis(display_frame, camera_matrix_, dist_coeffs_, 
                                   rvecs[i], tvecs[i], marker_size_ * 0.5f);
             }
             
             // If we have at least 3 landmarks, perform triangulation
             if (detected_ids.size() >= 3 || force_absolute_pose_) {
                 performTriangulation(detected_ids, marker_corners, display_frame);
                 force_absolute_pose_ = false;
             }
         }
         
         // Alternative: Try basic feature detection if ArUco fails
         if (detected_ids.size() < 3 && (force_absolute_pose_ || detected_ids.empty())) {
             detectFeaturesWithORB(rgb_image, display_frame);
         }
         
         // Alternative: Try YOLO detection if other methods fail
         if (detected_ids.size() < 3 && (force_absolute_pose_ || detected_ids.empty())) {
             detectLandmarksWithYOLO(rgb_image, display_frame);
         }
         
         // Publish the image with detections
         sensor_msgs::ImagePtr output_msg = 
             cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_frame).toImageMsg();
         image_pub_.publish(output_msg);
         
         // Display image if in verbose mode
         if (verbose_mode_) {
             cv::imshow("Landmark Detection", display_frame);
             cv::waitKey(1);
         }
     }
     
     void detectFeaturesWithORB(const cv::Mat& frame, cv::Mat& display_frame) {
         // Detect keypoints using ORB
         std::vector<cv::KeyPoint> keypoints;
         feature_detector_->detect(frame, keypoints);
         
         // Draw keypoints for visualization
         cv::drawKeypoints(display_frame, keypoints, display_frame, 
                          cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
         
         // In a real implementation, you would:
         // 1. Extract descriptors from detected keypoints
         // 2. Match descriptors against reference landmarks
         // 3. Filter matches to find reliable landmark detections
         // 4. Use the positions of the matched landmarks for triangulation
         
         if (verbose_mode_) {
             ROS_INFO("ORB detector found %zu keypoints", keypoints.size());
         }
         
         // Add note on display frame
         cv::putText(display_frame, "Using ORB features (ArUco markers not found)",
                    cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                    cv::Scalar(0, 200, 200), 2);
     }
     
     void detectLandmarksWithYOLO(const cv::Mat& frame, cv::Mat& display_frame) {
         try {
             // Create a 4D blob from the frame
             cv::Mat blob;
             cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(416, 416), 
                                  cv::Scalar(0, 0, 0), true, false);
             
             // Set the input blob
             yolo_net_.setInput(blob);
             
             // Get output layer names
             std::vector<std::string> outLayerNames = yolo_net_.getUnconnectedOutLayersNames();
             
             // Forward pass and get detections
             std::vector<cv::Mat> detections;
             yolo_net_.forward(detections, outLayerNames);
             
             // Process detections
             float confThreshold = 0.5;
             float nmsThreshold = 0.4;
             
             std::vector<int> classIds;
             std::vector<float> confidences;
             std::vector<cv::Rect> boxes;
             
             for (const auto& detection : detections) {
                 for (int i = 0; i < detection.rows; ++i) {
                     cv::Mat scores = detection.row(i).colRange(5, detection.cols);
                     cv::Point classIdPoint;
                     double confidence;
                     
                     minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                     
                     if (confidence > confThreshold) {
                         int centerX = static_cast<int>(detection.at<float>(i, 0) * frame.cols);
                         int centerY = static_cast<int>(detection.at<float>(i, 1) * frame.rows);
                         int width = static_cast<int>(detection.at<float>(i, 2) * frame.cols);
                         int height = static_cast<int>(detection.at<float>(i, 3) * frame.rows);
                         
                         int left = centerX - width / 2;
                         int top = centerY - height / 2;
                         
                         classIds.push_back(classIdPoint.x);
                         confidences.push_back(static_cast<float>(confidence));
                         boxes.push_back(cv::Rect(left, top, width, height));
                     }
                 }
             }
             
             // Apply non-maximum suppression
             std::vector<int> indices;
             cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
             
             // Draw bounding boxes
             for (size_t i = 0; i < indices.size(); ++i) {
                 int idx = indices[i];
                 cv::Rect box = boxes[idx];
                 
                 rectangle(display_frame, box, cv::Scalar(0, 0, 255), 2);
                 std::string label = std::to_string(classIds[idx]) + ": " + 
                                  std::to_string(confidences[idx]);
                 
                 int baseLine;
                 cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 
                                             0.5, 1, &baseLine);
                 
                 rectangle(display_frame, 
                          cv::Point(box.x, box.y - labelSize.height), 
                          cv::Point(box.x + labelSize.width, box.y + baseLine), 
                          cv::Scalar(255, 255, 255), cv::FILLED);
                          
                 putText(display_frame, label, cv::Point(box.x, box.y), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
             }
             
             // Add note on display frame
             cv::putText(display_frame, "Using YOLO detection (ArUco markers not found)",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        cv::Scalar(0, 0, 255), 2);
             
             if (verbose_mode_) {
                 ROS_INFO("YOLO detected %zu objects", indices.size());
             }
             
         } catch (const cv::Exception& e) {
             ROS_ERROR("YOLO detection error: %s", e.what());
             
             // Add error message to display frame
             cv::putText(display_frame, "YOLO detection failed: " + std::string(e.what()),
                        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        cv::Scalar(0, 0, 255), 2);
         }
     }
     
     void performTriangulation(const std::vector<int>& detected_ids, 
                              const std::vector<std::vector<cv::Point2f>>& marker_corners,
                              cv::Mat& display_frame) {
         // Map detected ArUco IDs to our landmark configuration
         std::vector<Landmark> detected_landmarks;
         std::vector<cv::Point2f> landmark_centers;
         
         for (size_t i = 0; i < detected_ids.size(); ++i) {
             int id = detected_ids[i];
             
             // Find this ID in our landmarks configuration
             auto it = std::find_if(landmarks_.begin(), landmarks_.end(),
                                  [id](const Landmark& l) { return l.id == id; });
             
             if (it != landmarks_.end()) {
                 detected_landmarks.push_back(*it);
                 
                 // Calculate the center of the marker
                 cv::Point2f center(0, 0);
                 for (const auto& corner : marker_corners[i]) {
                     center += corner;
                 }
                 center *= (1.0 / 4.0);
                 landmark_centers.push_back(center);
                 
                 // Draw the ID and coordinates on the display frame
                 std::string text = "ID: " + std::to_string(id) + 
                                 " (" + std::to_string(it->x) + ", " + 
                                 std::to_string(it->y) + ")";
                 cv::putText(display_frame, text, center, 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
             }
         }
         
         // Need at least 3 landmarks for triangulation
         if (detected_landmarks.size() >= 3) {
             // Select 3 landmarks
             Landmark& l1 = detected_landmarks[0];
             Landmark& l2 = detected_landmarks[1];
             Landmark& l3 = detected_landmarks[2];
             
             cv::Point2f& c1 = landmark_centers[0];
             cv::Point2f& c2 = landmark_centers[1];
             cv::Point2f& c3 = landmark_centers[2];
             
             // Calculate angles between landmarks
             // Convert pixel coordinates to angles based on camera intrinsics
             
             // Assuming optical center from camera matrix
             cv::Point2f image_center(camera_matrix_.at<double>(0, 2), 
                                    camera_matrix_.at<double>(1, 2));
             
             // Calculate vectors from image center to landmarks
             cv::Point2f v1 = c1 - image_center;
             cv::Point2f v2 = c2 - image_center;
             cv::Point2f v3 = c3 - image_center;
             
             // Convert to angles using camera intrinsics
             // For proper angle calculation, we need to convert to 3D rays
             // Focal length from camera matrix
             double fx = camera_matrix_.at<double>(0, 0);
             double fy = camera_matrix_.at<double>(1, 1);
             
             // Normalize by focal length
             cv::Point3f ray1(v1.x / fx, v1.y / fy, 1.0);
             cv::Point3f ray2(v2.x / fx, v2.y / fy, 1.0);
             cv::Point3f ray3(v3.x / fx, v3.y / fy, 1.0);
             
             // Normalize rays
             auto normalize3d = [](cv::Point3f& v) {
                 float norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                 if (norm > 0) v = v * (1.0f / norm);
             };
             
             normalize3d(ray1);
             normalize3d(ray2);
             normalize3d(ray3);
             
             // Calculate angles between rays (dot product)
             auto angle_between_rays = [](const cv::Point3f& a, const cv::Point3f& b) {
                 double dot = a.x * b.x + a.y * b.y + a.z * b.z;
                 // Clamp to avoid numerical issues
                 dot = std::max(-1.0, std::min(1.0, dot));
                 return std::acos(dot) * (180.0 / M_PI);
             };
             
             double alpha1 = angle_between_rays(ray1, ray2);
             double alpha2 = angle_between_rays(ray2, ray3);
             
             if (verbose_mode_) {
                 ROS_INFO("Angle between landmarks 1 and 2: %f degrees", alpha1);
                 ROS_INFO("Angle between landmarks 2 and 3: %f degrees", alpha2);
             }
             
             // Now use our triangulation function to compute robot position
             double xc1a, yc1a, xc1b, yc1b, r1;
             double xc2a, yc2a, xc2b, yc2b, r2;
             double x1_intersection, y1_intersection, x2_intersection, y2_intersection;
             
             // Call the triangulation functions
             circle_centre(l2.x, l2.y, l1.x, l1.y, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
             circle_centre(l3.x, l3.y, l2.x, l2.y, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);
             
             int result = circle_circle_intersection(
                 xc1a, yc1a, r1, xc2a, yc2a, r2,
                 &x1_intersection, &y1_intersection, 
                 &x2_intersection, &y2_intersection
             );
             
             if (result == 1) {
                 // Determine which intersection point is the robot position
                 // (the other might be a landmark position or another point on the circle)
                 double tolerance = 0.001;
                 double xr, yr;
                 
                 if ((std::fabs(x1_intersection - l1.x) < tolerance && std::fabs(y1_intersection - l1.y) < tolerance) ||
                     (std::fabs(x1_intersection - l2.x) < tolerance && std::fabs(y1_intersection - l2.y) < tolerance) ||
                     (std::fabs(x1_intersection - l3.x) < tolerance && std::fabs(y1_intersection - l3.y) < tolerance)) {
                     xr = x2_intersection;
                     yr = y2_intersection;
                 } else {
                     xr = x1_intersection;
                     yr = y1_intersection;
                 }
                 
                 // Calculate orientation based on the line of sight to landmark 1
                 // and adjusting for the head yaw angle
                 double dx = l1.x - xr;
                 double dy = l1.y - yr;
                 double landmark_angle = std::atan2(dy, dx);
                 
                 // Adjust for head yaw to get robot base orientation
                 double theta = landmark_angle - head_yaw_angle_;
                 
                 // Update pose with triangulation results
                 updatePose(xr, yr, theta);
                 
                 // Draw the calculated position on the display frame
                 cv::Point2f img_center(display_frame.cols / 2.0f, display_frame.rows / 2.0f);
                 cv::circle(display_frame, img_center, 10, cv::Scalar(0, 255, 0), -1);
                 
                 std::string pose_text = "Robot @ (" + std::to_string(xr) + ", " + 
                                       std::to_string(yr) + ", " + 
                                       std::to_string(theta * 180.0 / M_PI) + "deg)";
                 cv::putText(display_frame, pose_text,
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                            cv::Scalar(0, 255, 0), 2);
                 
                 if (verbose_mode_) {
                     ROS_INFO("Triangulation successful - Position: (%f, %f), Orientation: %f deg", 
                            xr, yr, theta * 180.0 / M_PI);
                 }
             } else {
                 if (verbose_mode_) {
                     ROS_WARN("Triangulation failed - Circles do not intersect");
                 }
             }
         } else {
             if (verbose_mode_ && force_absolute_pose_) {
                 ROS_WARN("Not enough landmarks detected for triangulation (need at least 3, found %zu)", 
                        detected_landmarks.size());
             }
         }
     }
     
     void updatePose(double x, double y, double theta) {
         std::lock_guard<std::mutex> lock(pose_mutex_);
         
         // Update the pose
         x_ = x;
         y_ = y;
         theta_ = theta;
         
         // Normalize theta to [-pi, pi]
         while (theta_ > M_PI) theta_ -= 2 * M_PI;
         while (theta_ < -M_PI) theta_ += 2 * M_PI;
         
         // Publish the updated pose
         publishPose();
         
         if (verbose_mode_) {
             ROS_INFO("Pose updated by triangulation - Position: (%f, %f), Orientation: %f", 
                    x_, y_, theta_);
         }
     }
     
     void publishPose() {
         geometry_msgs::PoseStamped pose_msg;
         
         pose_msg.header.stamp = ros::Time::now();
         pose_msg.header.frame_id = "map";
         
         pose_msg.pose.position.x = x_;
         pose_msg.pose.position.y = y_;
         pose_msg.pose.position.z = 0.0;
         
         tf2::Quaternion q;
         q.setRPY(0, 0, theta_);
         pose_msg.pose.orientation = tf2::toMsg(q);
         
         pose_pub_.publish(pose_msg);
     }
     
     bool resetPoseCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
         ROS_INFO("Reset pose service called");
         
         // Force an absolute pose estimation using triangulation
         force_absolute_pose_ = true;
         
         return true;
     }
     
     void run() {
         ros::Rate rate(10); // 10 Hz
         
         while (ros::ok()) {
             ros::spinOnce();
             rate.sleep();
         }
     }
 };
 
 int main(int argc, char** argv) {
     ros::init(argc, argv, "robot_localization_node");
     
     RobotLocalizationNode node;
     node.run();
     
     return 0;
 }