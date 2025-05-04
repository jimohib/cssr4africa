#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>
#include <robot_localization/ResetPose.h>
#include "robotLocalization/robotLocalizationInterface.h"

class RobotLocalizationNode {
public:
    RobotLocalizationNode() : nh_("~"), it_(nh_) {
        // Load parameters
        nh_.param("verbose", verbose_, false);
        nh_.param("use_depth", use_depth_, false);
        nh_.param("reset_interval", reset_interval_, 10.0); // seconds
        nh_.param("config_file", config_file_, std::string("config/landmarks.yaml"));
        nh_.param("topics_file", topics_file_, std::string("data/pepperTopics.dat"));

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

        // Publishers
        pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/robotLocalization/pose", 10);
        image_pub_ = it_.advertise("/robotLocalization/marker_image", 10);

        // Service
        reset_srv_ = nh_.advertiseService("/robotLocalization/reset_pose", &RobotLocalizationNode::resetPoseCallback, this);

        // Initialize pose
        current_pose_.x = 0.0;
        current_pose_.y = 0.0;
        current_pose_.theta = 0.0;

        // Timer for periodic reset
        reset_timer_ = nh_.createTimer(ros::Duration(reset_interval_), &RobotLocalizationNode::resetTimerCallback, this);

        // Camera parameters (adjust as needed)
        fx_ = 615.0; // Focal length in x (pixels)
        fy_ = 615.0; // Focal length in y (pixels)
        cx_ = 320.0; // Principal point x (pixels)
        cy_ = 240.0; // Principal point y (pixels)

        ROS_INFO("Robot Localization Node initialized");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber odom_sub_, imu_sub_, joint_sub_;
    image_transport::Subscriber image_sub_, depth_sub_;
    ros::Publisher pose_pub_;
    image_transport::Publisher image_pub_;
    ros::ServiceServer reset_srv_;
    ros::Timer reset_timer_;

    bool verbose_, use_depth_;
    double reset_interval_;
    std::string config_file_, topics_file_;
    geometry_msgs::Pose2D current_pose_;
    std::map<int, std::pair<double, double>> landmarks_; // ID -> (x, y)
    std::map<std::string, std::string> topic_map_;
    cv::Mat latest_image_, latest_depth_;
    double head_yaw_ = 0.0;
    double fx_, fy_, cx_, cy_; // Camera intrinsics

    void loadTopicNames() {
        std::ifstream file(topics_file_);
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
    }

    void loadLandmarks() {
        YAML::Node config = YAML::LoadFile(config_file_);
        for (const auto& marker : config["landmarks"]) {
            int id = marker["id"].as<int>();
            double x = marker["x"].as<double>();
            double y = marker["y"].as<double>();
            landmarks_[id] = {x, y};
        }
        ROS_INFO("Loaded %zu landmarks from %s", landmarks_.size(), config_file_.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Update pose using odometry
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;

        publishPose();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Optionally refine yaw using IMU angular velocity
        // For simplicity, we rely on odometry for now
    }

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // Assume "HeadYaw" is the joint name for yaw
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "HeadYaw") { // Adjust name as per robot
                head_yaw_ = msg->position[i];
                break;
            }
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            latest_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    bool resetPoseCallback(robot_localization::ResetPose::Request& req, robot_localization::ResetPose::Response& res) {
        if (computeAbsolutePose()) {
            res.success = true;
            ROS_INFO("Pose reset successfully");
        } else {
            res.success = false;
            ROS_WARN("Failed to reset pose");
        }
        return true;
    }

    void dwim::EventTime reset_timer_;
    void resetTimerCallback(const ros::TimerEvent& event) {
        computeAbsolutePose();
    }

    bool computeAbsolutePose() {
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

            if (marker_ids.size() < 3) {
                ROS_WARN("Detected %zu markers, need at least 3", marker_ids.size());
                return false;
            }

            // Draw bounding boxes
            cv::Mat output_image = latest_image_.clone();
            cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);

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
            if (landmarks_.find(id1) == landmarks_.end() || landmarks_.find(id2) == landmarks_.end() || landmarks_.find(id3) == landmarks_.end()) {
                ROS_WARN("Unknown marker IDs detected");
                return false;
            }

            double x1 = landmarks_[id1].first, y1 = landmarks_[id1].second;
            double x2 = landmarks_[id2].first, y2 = landmarks_[id2].second;
            double x3 = landmarks_[id3].first, y3 = landmarks_[id3].second;

            // Compute angles alpha1 (between markers 1 and 2) and alpha2 (between markers 2 and 3)
            double alpha1 = computeAngle(marker_centers[0], marker_centers[1]);
            double alpha2 = computeAngle(marker_centers[1], marker_centers[2]);

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
            current_pose_.x = xr;
            current_pose_.y = yr;
            current_pose_.theta = theta;

            // Publish image
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
            image_pub_.publish(img_msg);

            // Display in verbose mode
            if (verbose_) {
                ROS_INFO("Robot Pose: x=%.3f, y=%.3f, theta=%.3f", xr, yr, theta);
                cv::imshow("ArUco Markers", output_image);
                cv::waitKey(1);
            }

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
            if (landmarks_.find(marker_ids[i]) != landmarks_.end() && !std::isnan(distance)) {
                markers.push_back({marker_ids[i], landmarks_[marker_ids[i]].first, landmarks_[marker_ids[i]].second, distance});
            }
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
    
        // Compute yaw (same as triangulation)
        double theta = computeYaw({marker_corners[0][0].x, marker_corners[0][0].y}, x1, y1, xr, yr);
    
        // Update pose
        current_pose_.x = xr;
        current_pose_.y = yr;
        current_pose_.theta = theta;
    
        // Publish image (same as triangulation)
        cv::Mat output_image = latest_image_.clone();
        cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids);
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
        image_pub_.publish(img_msg);
    
        if (verbose_) {
            ROS_INFO("Robot Pose (Depth): x=%.3f, y=%.3f, theta=%.3f", xr, yr, theta);
            cv::imshow("ArUco Markers", output_image);
            cv::waitKey(1);
        }
    
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
        // Adjust for head yaw
        double image_angle = (marker_center.first - cx_) / fx_;
        return world_angle - head_yaw_ - image_angle;
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