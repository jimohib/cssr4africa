/**
 * Robot Localization Node
 * 
 * This node implements robot localization using both relative (odometry/IMU) and 
 * absolute (triangulation) positioning methods.
 * 
 * Author: [Your Name]
 * Date: [Current Date]
 */

 #include <ros/ros.h>
 #include <ros/package.h>
 #include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Image.h>
 #include <sensor_msgs/Imu.h>
 #include <sensor_msgs/JointState.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/Pose2D.h>
 #include <cv_bridge/cv_bridge.h>
 #include <std_srvs/Trigger.h>
 #include <opencv2/opencv.hpp>
 #include <opencv2/aruco.hpp>
 #include <opencv2/features2d.hpp>
 #include <opencv2/calib3d.hpp>
 #include <fstream>
 #include <yaml-cpp/yaml.h>
 #include <tf2/LinearMath/Quaternion.h>
 #include <tf2/LinearMath/Matrix3x3.h>
 
 // Landmark struct to store landmark information
 struct Landmark {
     int id;
     double x;
     double y;
     std::string name;
 };
 
 class RobotLocalizationNode {
 private:
     // ROS Node Handle
     ros::NodeHandle nh_;
     
     // Publishers and Subscribers
     ros::Subscriber camera_sub_;
     ros::Subscriber odom_sub_;
     ros::Subscriber imu_sub_;
     ros::Subscriber joint_state_sub_;
     ros::Publisher pose_pub_;
     ros::Publisher image_pub_;
     ros::ServiceServer reset_pose_service_;
     
     // Parameters
     std::string camera_topic_;
     std::string odom_topic_;
     std::string imu_topic_;
     std::string joint_states_topic_;
     std::string pose_topic_;
     std::string landmarks_config_file_;
     bool verbose_mode_;
     
     // State variables
     geometry_msgs::Pose2D current_pose_;
     ros::Time last_update_time_;
     double head_yaw_angle_;
     
     // ArUco detection
     cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
     cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
     
     // Landmarks information
     std::vector<Landmark> landmarks_;
     
     // Camera calibration parameters
     cv::Mat camera_matrix_;
     cv::Mat dist_coeffs_;
     
     // SIFT detector
     cv::Ptr<cv::SIFT> sift_detector_;
 
 public:
     RobotLocalizationNode() : nh_("~") {
         // Initialize ROS parameters
         initializeParameters();
         
         // Load landmarks from configuration file
         loadLandmarks();
         
         // Initialize ArUco detector
         aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
         aruco_params_ = cv::aruco::DetectorParameters::create();
         
         // Initialize SIFT detector
         sift_detector_ = cv::SIFT::create();
         
         // Initialize camera calibration parameters (these should be loaded from a calibration file)
         camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
         dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
         
         // Initialize robot pose
         current_pose_.x = 0.0;
         current_pose_.y = 0.0;
         current_pose_.theta = 0.0;
         last_update_time_ = ros::Time::now();
         head_yaw_angle_ = 0.0;
         
         // Subscribe to topics
         camera_sub_ = nh_.subscribe(camera_topic_, 1, &RobotLocalizationNode::cameraCallback, this);
         odom_sub_ = nh_.subscribe(odom_topic_, 1, &RobotLocalizationNode::odomCallback, this);
         imu_sub_ = nh_.subscribe(imu_topic_, 1, &RobotLocalizationNode::imuCallback, this);
         joint_state_sub_ = nh_.subscribe(joint_states_topic_, 1, &RobotLocalizationNode::jointStateCallback, this);
         
         // Advertise publishers
         pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>(pose_topic_, 10);
         image_pub_ = nh_.advertise<sensor_msgs::Image>("/robotLocalization/debug_image", 1);
         
         // Advertise services
         reset_pose_service_ = nh_.advertiseService("/robotLocalization/reset_pose", 
                                                   &RobotLocalizationNode::resetPoseCallback, this);
         
         ROS_INFO("Robot Localization Node initialized");
     }
     
     /**
      * Initialize parameters from parameter server or default values
      */
     void initializeParameters() {
         nh_.param<std::string>("camera_topic", camera_topic_, "/camera/color/image_raw");
         nh_.param<std::string>("odom_topic", odom_topic_, "/naoqi_driver/odom");
         nh_.param<std::string>("imu_topic", imu_topic_, "/naoqi_driver/imu/base");
         nh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
         nh_.param<std::string>("pose_topic", pose_topic_, "/robotLocalization/pose");
         nh_.param<std::string>("topics_file", topics_file_, "pepperTopics.dat");
         nh_.param<std::string>("landmarks_config", landmarks_config_file_, "landmarks.yaml");
         nh_.param<bool>("verbose", verbose_mode_, false);
         
         // Additional parameter loading from config file if needed
         loadTopicsFromConfigFile();
     }
     
     /**
      * Load topic names from configuration file
      */
     void loadTopicsFromConfigFile() {
         std::string package_path = ros::package::getPath("robot_localization");
         std::string config_path = package_path + "/config/topics.yaml";
         
         try {
             YAML::Node config = YAML::LoadFile(config_path);
             
             if (config["camera_topic"]) {
                 camera_topic_ = config["camera_topic"].as<std::string>();
             }
             if (config["odom_topic"]) {
                 odom_topic_ = config["odom_topic"].as<std::string>();
             }
             if (config["imu_topic"]) {
                 imu_topic_ = config["imu_topic"].as<std::string>();
             }
             if (config["joint_states_topic"]) {
                 joint_states_topic_ = config["joint_states_topic"].as<std::string>();
             }
             
             ROS_INFO("Loaded topic configuration from file");
         } catch (const std::exception& e) {
             ROS_WARN("Failed to load topic configuration: %s", e.what());
             ROS_WARN("Using default topic names");
         }



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
     
     /**
      * Load landmarks from configuration file
      */
     void loadLandmarks() {
         std::string package_path = ros::package::getPath("robot_localization");
         std::string landmarks_path = package_path + "/config/" + landmarks_config_file_;
         
         try {
             YAML::Node config = YAML::LoadFile(landmarks_path);
             
             for (const auto& landmark : config["landmarks"]) {
                 Landmark lm;
                 lm.id = landmark["id"].as<int>();
                 lm.x = landmark["x"].as<double>();
                 lm.y = landmark["y"].as<double>();
                 lm.name = landmark["name"].as<std::string>();
                 
                 landmarks_.push_back(lm);
                 
                 ROS_INFO("Loaded landmark: %s (ID: %d) at position (%f, %f)", 
                          lm.name.c_str(), lm.id, lm.x, lm.y);
             }
         } catch (const std::exception& e) {
             ROS_ERROR("Failed to load landmarks: %s", e.what());
         }
     }
     
     /**
      * Callback for camera images
      */
     void cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
         try {
             // Convert ROS image to OpenCV format
             cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
             cv::Mat image = cv_ptr->image;
             
             // Process image and detect landmarks
             std::vector<cv::Point2f> landmark_pixels;
             std::vector<int> landmark_ids;
             std::vector<Landmark> detected_landmarks;
             
             // Detect ArUco markers and SIFT features
             detectLandmarks(image, landmark_pixels, landmark_ids, detected_landmarks);
             
             // If we have detected at least 3 landmarks, perform triangulation
             if (detected_landmarks.size() >= 3) {
                 // Use the first 3 landmarks for triangulation
                 performTriangulation(detected_landmarks, landmark_pixels);
             }
             
             // Draw landmark bounding boxes and pose information on image
             drawLandmarksAndPose(image, landmark_pixels, landmark_ids);
             
             // Publish processed image
             sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
             image_pub_.publish(img_msg);
             
             // Display image in verbose mode
             if (verbose_mode_) {
                 cv::imshow("Robot Localization", image);
                 cv::waitKey(1);
             }
             
         } catch (cv_bridge::Exception& e) {
             ROS_ERROR("CV bridge exception: %s", e.what());
         }
     }
     
     /**
      * Callback for odometry data
      */
     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
         // Update relative pose based on odometry
         ros::Time current_time = ros::Time::now();
         double dt = (current_time - last_update_time_).toSec();
         
         // Extract position and orientation from odometry
         double x = msg->pose.pose.position.x;
         double y = msg->pose.pose.position.y;
         
         // Convert quaternion to Euler angles
         tf2::Quaternion q(
             msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z,
             msg->pose.pose.orientation.w);
         tf2::Matrix3x3 m(q);
         double roll, pitch, yaw;
         m.getRPY(roll, pitch, yaw);
         
         // Update current pose (simple update for now, could be improved with proper fusion)
         current_pose_.x = x;
         current_pose_.y = y;
         current_pose_.theta = yaw;
         
         // Publish updated pose
         publishPose();
         
         // Update time
         last_update_time_ = current_time;
         
         if (verbose_mode_) {
             ROS_INFO("Relative pose update: x=%.2f, y=%.2f, theta=%.2f", 
                      current_pose_.x, current_pose_.y, current_pose_.theta);
         }
     }
     
     /**
      * Callback for IMU data
      */
     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
         // Process IMU data for orientation and acceleration
         // This would be used for complementary filtering with odometry
         // Not implemented in detail in this example
     }
     
     /**
      * Callback for joint state data
      */
     void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
         // Extract head yaw angle from joint states
         for (size_t i = 0; i < msg->name.size(); i++) {
             if (msg->name[i] == "HeadYaw") {  // Adjust for actual joint name in Pepper
                 head_yaw_angle_ = msg->position[i];
                 break;
             }
         }
     }
     
     /**
      * Callback for reset pose service
      */
     bool resetPoseCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
         ROS_INFO("Reset pose service called");
         
         // Perform absolute position estimation using triangulation
         // This will be called when we need to reset the pose due to drift
         
         res.success = false;
         res.message = "Waiting for next camera frame to reset pose";
         
         // The actual reset will happen in the next camera callback
         // if enough landmarks are detected
         
         return true;
     }
     
     /**
      * Detect landmarks in the image using ArUco markers and SIFT features
      */
     void detectLandmarks(const cv::Mat& image, 
                          std::vector<cv::Point2f>& landmark_pixels,
                          std::vector<int>& landmark_ids,
                          std::vector<Landmark>& detected_landmarks) {
         // Detect ArUco markers
         std::vector<int> ids;
         std::vector<std::vector<cv::Point2f>> corners;
         cv::aruco::detectMarkers(image, aruco_dict_, corners, ids, aruco_params_);
         
         // Process detected markers
         for (size_t i = 0; i < ids.size(); i++) {
             int marker_id = ids[i];
             
             // Find corresponding landmark
             for (const Landmark& lm : landmarks_) {
                 if (lm.id == marker_id) {
                     // Calculate center of the marker
                     cv::Point2f center(0, 0);
                     for (const auto& corner : corners[i]) {
                         center += corner;
                     }
                     center *= 1.0f / 4.0f;
                     
                     // Store landmark information
                     landmark_pixels.push_back(center);
                     landmark_ids.push_back(marker_id);
                     detected_landmarks.push_back(lm);
                     break;
                 }
             }
         }
         
         // Additional SIFT-based detection could be implemented here
         // This would be useful for detecting non-ArUco landmarks
     }
     
     /**
      * Perform triangulation to determine absolute position
      */
     void performTriangulation(const std::vector<Landmark>& detected_landmarks,
                               const std::vector<cv::Point2f>& landmark_pixels) {
         // Ensure we have at least 3 landmarks
         if (detected_landmarks.size() < 3) {
             return;
         }
         
         // Get the first three landmarks
         double x1 = detected_landmarks[0].x;
         double y1 = detected_landmarks[0].y;
         double x2 = detected_landmarks[1].x;
         double y2 = detected_landmarks[1].y;
         double x3 = detected_landmarks[2].x;
         double y3 = detected_landmarks[2].y;
         
         // Calculate the angles subtended by the landmarks from the robot's perspective
         double alpha1 = calculateSubtendedAngle(landmark_pixels[0], landmark_pixels[1]);
         double alpha2 = calculateSubtendedAngle(landmark_pixels[1], landmark_pixels[2]);
         
         // Calculate position using triangulation
         double xr, yr;
         triangulatePosition(x1, y1, x2, y2, x3, y3, alpha1, alpha2, xr, yr);
         
         // Update robot position
         current_pose_.x = xr;
         current_pose_.y = yr;
         
         // Calculate orientation based on landmark positions
         calculateOrientation(detected_landmarks[0], landmark_pixels[0]);
         
         // Publish updated pose
         publishPose();
         
         if (verbose_mode_) {
             ROS_INFO("Absolute pose update: x=%.2f, y=%.2f, theta=%.2f", 
                      current_pose_.x, current_pose_.y, current_pose_.theta);
         }
     }
     
     /**
      * Calculate the angle subtended by two points in the image
      */
     double calculateSubtendedAngle(const cv::Point2f& p1, const cv::Point2f& p2) {
         // Calculate vectors from camera center to the points
         // This is a simplified version - in reality, you need to use camera calibration
         cv::Point2f center(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
         cv::Point2f v1 = p1 - center;
         cv::Point2f v2 = p2 - center;
         
         // Calculate angle between vectors
         double dot = v1.x * v2.x + v1.y * v2.y;
         double mag1 = sqrt(v1.x * v1.x + v1.y * v1.y);
         double mag2 = sqrt(v2.x * v2.x + v2.y * v2.y);
         
         double angle = acos(dot / (mag1 * mag2)) * 180.0 / M_PI;
         
         return angle;
     }
     
     /**
      * Calculate orientation based on landmark positions
      */
     void calculateOrientation(const Landmark& lm, const cv::Point2f& pixel) {
         // Calculate direction in world coordinates
         double dx = lm.x - current_pose_.x;
         double dy = lm.y - current_pose_.y;
         double world_angle = atan2(dy, dx);
         
         // Calculate direction in camera coordinates
         cv::Point2f center(camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
         cv::Point2f v = pixel - center;
         double camera_angle = atan2(v.y, v.x);
         
         // Calculate robot orientation
         // Need to adjust for camera orientation relative to robot
         current_pose_.theta = world_angle - camera_angle - head_yaw_angle_;
         
         // Normalize angle to [-π, π]
         while (current_pose_.theta > M_PI) current_pose_.theta -= 2 * M_PI;
         while (current_pose_.theta <= -M_PI) current_pose_.theta += 2 * M_PI;
     }
     
     /**
      * Draw landmarks and pose information on the image
      */
     void drawLandmarksAndPose(cv::Mat& image, 
                               const std::vector<cv::Point2f>& landmark_pixels,
                               const std::vector<int>& landmark_ids) {
         // Draw detected landmarks
         for (size_t i = 0; i < landmark_pixels.size(); i++) {
             cv::circle(image, landmark_pixels[i], 5, cv::Scalar(0, 255, 0), -1);
             cv::putText(image, "ID: " + std::to_string(landmark_ids[i]), 
                         landmark_pixels[i] + cv::Point2f(10, 10),
                         cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
         }
         
         // Draw current pose information
         std::string pose_text = "Position: (" + std::to_string(current_pose_.x).substr(0, 5) + 
                                ", " + std::to_string(current_pose_.y).substr(0, 5) + 
                                ") Orientation: " + std::to_string(current_pose_.theta * 180.0 / M_PI).substr(0, 5) + " deg";
         cv::putText(image, pose_text, cv::Point(10, 30), 
                     cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
     }
     
     /**
      * Publish current pose
      */
     void publishPose() {
         geometry_msgs::Pose2D pose_msg;
         pose_msg.x = current_pose_.x;
         pose_msg.y = current_pose_.y;
         pose_msg.theta = current_pose_.theta;
         pose_pub_.publish(pose_msg);
     }
     
     /**
      * Triangulate position using the algorithm from the provided code
      */
     void triangulatePosition(double x1, double y1, double x2, double y2, double x3, double y3,
                             double alpha1, double alpha2, double& xr, double& yr) {
         double xc1a, yc1a, xc1b, yc1b, r1;
         double xc2a, yc2a, xc2b, yc2b, r2;
         double x1_intersection, y1_intersection, x2_intersection, y2_intersection;
         double tolerance = 0.001;
         
         // Find circles from the first pair of landmarks
         circleCentre(x2, y2, x1, y1, alpha1, &xc1a, &yc1a, &xc1b, &yc1b, &r1);
         
         // Find circles from the second pair of landmarks
         circleCentre(x3, y3, x2, y2, alpha2, &xc2a, &yc2a, &xc2b, &yc2b, &r2);
         
         // Find intersection of the two circles
         int result = circleCircleIntersection(xc1a, yc1a, r1, xc2a, yc2a, r2,
                                              &x1_intersection, &y1_intersection,
                                              &x2_intersection, &y2_intersection);
         
         if (result == 1) {
             // One of the intersection points is a landmark
             // The other is the robot position
             if (((fabs(x1_intersection - x1) < tolerance) && (fabs(y1_intersection - y1) < tolerance)) ||
                 ((fabs(x1_intersection - x2) < tolerance) && (fabs(y1_intersection - y2) < tolerance)) ||
                 ((fabs(x1_intersection - x3) < tolerance) && (fabs(y1_intersection - y3) < tolerance))) {
                 xr = x2_intersection;
                 yr = y2_intersection;
             } else {
                 xr = x1_intersection;
                 yr = y1_intersection;
             }
         } else {
             ROS_WARN("Triangulation failed: circles do not intersect");
             // Keep the current position estimate
         }
     }
     
     /**
      * Find the center of a circle given a chord and subtended angle
      * Ported from the provided code
      */
     int circleCentre(double x1, double y1, double x2, double y2, double alpha,
                     double* xc1, double* yc1, double* xc2, double* yc2, double* r) {
         double d;
         double h;
         double theta;
         double beta;
         double delta_x;
         double delta_y;
         double alphar;
         double temp_x;
         double temp_y;
         
         alphar = M_PI * (alpha / 180.0); // convert to radians
         
         d = sqrt((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
         
         if (alpha == 0 || alpha == 180) {
             h = 0;
             *r = 0;
         } else {
             h = (d / 2) / tan(alphar);
             *r = (d / 2) / sin(alphar);
         }
         
         theta = atan2(y2-y1, x2-x1);
         beta = theta - (M_PI / 2);
         delta_x = h * cos(beta);
         delta_y = h * sin(beta);
         
         *xc1 = (x1 + x2)/2 - delta_x;
         *yc1 = (y1 + y2)/2 - delta_y;
         
         *xc2 = (x1 + x2)/2 + delta_x;
         *yc2 = (y1 + y2)/2 + delta_y;
         
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
     
     /**
      * Find the intersection of two circles
      * Ported from the provided code
      */
     int circleCircleIntersection(double x0, double y0, double r0,
                                 double x1, double y1, double r1,
                                 double* xi, double* yi,
                                 double* xi_prime, double* yi_prime) {
         double a, dx, dy, d, h, rx, ry;
         double x2, y2;
         
         dx = x1 - x0;
         dy = y1 - y0;
         
         d = hypot(dx, dy);
         
         if (d > (r0 + r1)) {
             return 0; // No solution, circles do not intersect
         }
         if (d < fabs(r0 - r1)) {
             return 0; // No solution, one circle is contained in the other
         }
         
         a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d);
         
         x2 = x0 + (dx * a/d);
         y2 = y0 + (dy * a/d);
         
         h = sqrt((r0*r0) - (a*a));
         
         rx = -dy * (h/d);
         ry = dx * (h/d);
         
         *xi = x2 + rx;
         *xi_prime = x2 - rx;
         *yi = y2 + ry;
         *yi_prime = y2 - ry;
         
         return 1;
     }
 };
 
 int main(int argc, char** argv) {
     ros::init(argc, argv, "robot_localization_node");
     
     RobotLocalizationNode node;
     
     ros::spin();
     
     return 0;
 }