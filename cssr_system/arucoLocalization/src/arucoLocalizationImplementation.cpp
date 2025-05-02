#include <ros/ros.h>
#include <sensor_msgs/Image.h>
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
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>

// Include our triangulation algorithm header
#include <assignment1/dvernon.h>

class RobotLocalizationNode {
private:
    // ROS node handle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // ROS subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber camera_sub_;
    ros::Subscriber head_yaw_sub_;
    
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
        double x, y;  // Position in world coordinates
    };
    std::vector<Landmark> landmarks_;
    
    // Parameters
    std::string config_file_;
    std::string topics_file_;
    bool verbose_mode_;
    
    // OpenCV and detection-related
    cv::Ptr<cv::Feature2D> sift_detector_;
    cv::dnn::Net yolo_net_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
    
    // IMU bias correction
    double gyro_bias_z_;
    
    // Head yaw angle
    double head_yaw_angle_;

public:
    RobotLocalizationNode() : private_nh_("~") {
        // Load parameters
        private_nh_.param<std::string>("config_file", config_file_, "landmarks.yaml");
        private_nh_.param<std::string>("topics_file", topics_file_, "topics.txt");
        private_nh_.param<bool>("verbose", verbose_mode_, false);
        
        // Initialize pose
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        
        // Initialize IMU bias
        gyro_bias_z_ = 0.0;
        
        // Initialize head yaw
        head_yaw_angle_ = 0.0;
        
        // Load landmarks configuration
        loadLandmarksConfig();
        
        // Setup subscribers and publishers based on topics file
        setupTopics();
        
        // Initialize SIFT detector
        sift_detector_ = cv::SIFT::create();
        
        // Initialize YOLO network
        initializeYOLO();
        
        // Initialize ArUco detector
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        aruco_params_ = cv::aruco::DetectorParameters::create();
        
        // Create service
        reset_pose_service_ = nh_.advertiseService("robotLocalization/reset_pose", 
                                                   &RobotLocalizationNode::resetPoseCallback, this);
        
        ROS_INFO("Robot Localization Node initialized");
    }
    
    ~RobotLocalizationNode() {
        if (verbose_mode_) {
            cv::destroyAllWindows();
        }
    }
    
    void loadLandmarksConfig() {
        try {
            YAML::Node config = YAML::LoadFile(config_file_);
            
            if (config["landmarks"]) {
                auto landmarks_config = config["landmarks"];
                for (const auto& landmark : landmarks_config) {
                    Landmark l;
                    l.id = landmark["id"].as<int>();
                    l.x = landmark["x"].as<double>();
                    l.y = landmark["y"].as<double>();
                    landmarks_.push_back(l);
                    
                    ROS_INFO("Loaded landmark ID: %d, Position: (%f, %f)", l.id, l.x, l.y);
                }
            } else {
                ROS_ERROR("No landmarks defined in configuration file");
            }
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Error loading landmarks configuration: %s", e.what());
        }
    }
    
    void setupTopics() {
        std::ifstream topics_file(topics_file_);
        if (!topics_file.is_open()) {
            ROS_ERROR("Could not open topics file: %s", topics_file_.c_str());
            return;
        }
        
        std::string line;
        std::map<std::string, std::string> topic_map;
        
        while (std::getline(topics_file, line)) {
            size_t delimiter_pos = line.find(':');
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
        
        // Set up subscribers
        if (topic_map.count("odometry")) {
            odom_sub_ = nh_.subscribe(topic_map["odometry"], 10, 
                               &RobotLocalizationNode::odometryCallback, this);
            ROS_INFO("Subscribed to odometry topic: %s", topic_map["odometry"].c_str());
        }
        
        if (topic_map.count("imu")) {
            imu_sub_ = nh_.subscribe(topic_map["imu"], 10, 
                             &RobotLocalizationNode::imuCallback, this);
            ROS_INFO("Subscribed to IMU topic: %s", topic_map["imu"].c_str());
        }
        
        if (topic_map.count("camera")) {
            camera_sub_ = nh_.subscribe(topic_map["camera"], 1, 
                                &RobotLocalizationNode::cameraCallback, this);
            ROS_INFO("Subscribed to camera topic: %s", topic_map["camera"].c_str());
        }
        
        if (topic_map.count("head_yaw")) {
            head_yaw_sub_ = nh_.subscribe(topic_map["head_yaw"], 10, 
                                  &RobotLocalizationNode::headYawCallback, this);
            ROS_INFO("Subscribed to head yaw topic: %s", topic_map["head_yaw"].c_str());
        }
        
        // Set up publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("robotLocalization/pose", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("robotLocalization/landmark_detection", 1);
    }
    
    void initializeYOLO() {
        try {
            // Load YOLO network
            yolo_net_ = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");
            yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            ROS_INFO("YOLO network initialized");
        } catch (const cv::Exception& e) {
            ROS_ERROR("Failed to initialize YOLO: %s", e.what());
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
    
    void headYawCallback(const std_msgs::Float64::ConstPtr& msg) {
        head_yaw_angle_ = msg->data;
        
        if (verbose_mode_) {
            ROS_INFO("Head yaw angle updated: %f", head_yaw_angle_);
        }
    }
    
    void cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            
            // Create a copy for visualization
            cv::Mat display_frame = frame.clone();
            
            // Detect ArUco markers
            std::vector<int> detected_ids;
            std::vector<std::vector<cv::Point2f>> marker_corners;
            
            cv::aruco::detectMarkers(frame, aruco_dict_, marker_corners, detected_ids, aruco_params_);
            
            // Draw detected markers
            if (detected_ids.size() > 0) {
                cv::aruco::drawDetectedMarkers(display_frame, marker_corners, detected_ids);
                
                // If we have at least 3 landmarks, perform triangulation
                if (detected_ids.size() >= 3) {
                    performTriangulation(detected_ids, marker_corners, display_frame);
                }
            }
            
            // Alternative: Try SIFT detection if ArUco fails
            if (detected_ids.size() < 3) {
                detectLandmarksWithSIFT(frame, display_frame);
            }
            
            // Alternative: Try YOLO detection if other methods fail
            if (detected_ids.size() < 3) {
                detectLandmarksWithYOLO(frame, display_frame);
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
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
    void detectLandmarksWithSIFT(const cv::Mat& frame, cv::Mat& display_frame) {
        // This is a simplified implementation - in practice you would need reference
        // images of your landmarks to match against
        
        // Detect keypoints
        std::vector<cv::KeyPoint> keypoints;
        sift_detector_->detect(frame, keypoints);
        
        // Draw keypoints for visualization
        cv::drawKeypoints(display_frame, keypoints, display_frame, 
                         cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        
        // In a real implementation, you would:
        // 1. Extract descriptors from detected keypoints
        // 2. Match descriptors against reference landmarks
        // 3. Filter matches to find reliable landmark detections
        // 4. Use the positions of the matched landmarks for triangulation
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
            
            // In a real implementation, you would need to:
            // 1. Filter detections to identify your specific landmarks
            // 2. Map detected objects to landmark IDs
            // 3. Use positions for triangulation
            
        } catch (const cv::Exception& e) {
            ROS_ERROR("YOLO detection error: %s", e.what());
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
            // This is a simplified approach - you would need to use proper camera calibration
            
            // Assuming optical center at image center
            cv::Point2f image_center(display_frame.cols / 2.0, display_frame.rows / 2.0);
            
            // Calculate vectors from image center to landmarks
            cv::Point2f v1 = c1 - image_center;
            cv::Point2f v2 = c2 - image_center;
            cv::Point2f v3 = c3 - image_center;
            
            // Normalize vectors
            auto normalize = [](cv::Point2f& v) {
                float length = std::sqrt(v.x * v.x + v.y * v.y);
                if (length > 0) v /= length;
            };
            
            normalize(v1);
            normalize(v2);
            normalize(v3);
            
            // Calculate angles between vectors
            auto angle_between = [](const cv::Point2f& a, const cv::Point2f& b) {
                return std::acos(a.x * b.x + a.y * b.y) * (180.0 / M_PI);
            };
            
            double alpha1 = angle_between(v1, v2);
            double alpha2 = angle_between(v2, v3);
            
            // Now use our triangulation function to compute robot position
            double xc1a, yc1a, xc1b, yc1b, r1;
            double xc2a, yc2a, xc2b, yc2b, r2;
            double x1_intersection, y1_intersection, x2_intersection, y2_intersection;
            
            // Convert angles to the format expected by the triangulation function
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
                // (this is a simplified approach)
                double theta = landmark_angle - head_yaw_angle_;
                
                // Update pose with triangulation results
                updatePose(xr, yr, theta);
                
                // Draw the calculated position on the display frame
                cv::circle(display_frame, image_center, 10, cv::Scalar(0, 255, 0), -1);
                cv::putText(display_frame, "Robot @ (" + std::to_string(xr) + ", " + 
                           std::to_string(yr) + ", " + std::to_string(theta * 180.0 / M_PI) + "deg)",
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                           cv::Scalar(0, 255, 0), 2);
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
        // This will happen on the next camera callback
        
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