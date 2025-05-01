#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map>

class MarkerVisualizer {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    ros::Timer timer_;
    
    std::map<int, geometry_msgs::Point> marker_positions_;
    std::map<int, double> marker_orientations_;
    std::string world_frame_;
    double marker_size_;

public:
    MarkerVisualizer() : private_nh_("~") {
        // Load parameters
        loadParameters();
        
        // Set up publishers
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1, true);
        
        // Set up timer for periodic visualization updates
        timer_ = nh_.createTimer(ros::Duration(1.0), &MarkerVisualizer::timerCallback, this);
        
        ROS_INFO("ArUco marker visualizer initialized");
    }
    
    void loadParameters() {
        // Frame ID
        private_nh_.param<std::string>("world_frame", world_frame_, "map");
        
        // Marker size
        private_nh_.param<double>("marker_size", marker_size_, 0.2);
        
        // Load marker positions
        XmlRpc::XmlRpcValue markers;
        private_nh_.getParam("markers", markers);
        if (markers.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            for (XmlRpc::XmlRpcValue::iterator it = markers.begin(); it != markers.end(); ++it) {
                int marker_id = std::stoi(it->first);
                XmlRpc::XmlRpcValue marker_data = it->second;
                
                if (marker_data.getType() == XmlRpc::XmlRpcValue::TypeArray && marker_data.size() == 3) {
                    geometry_msgs::Point point;
                    point.x = static_cast<double>(marker_data[0]);
                    point.y = static_cast<double>(marker_data[1]);
                    point.z = static_cast<double>(marker_data[2]);
                    
                    marker_positions_[marker_id] = point;
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
            }
        }
    }
    
    void timerCallback(const ros::TimerEvent&) {
        publishMarkers();
    }
    
    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        
        for (const auto& entry : marker_positions_) {
            int id = entry.first;
            const geometry_msgs::Point& pos = entry.second;
            
            // Create marker cube
            visualization_msgs::Marker marker;
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "aruco_markers";
            marker.id = id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // Set position
            marker.pose.position = pos;
            
            // Set orientation based on marker orientation if available
            double orientation_deg = 0.0;
            if (marker_orientations_.find(id) != marker_orientations_.end()) {
                orientation_deg = marker_orientations_[id];
            }
            
            // Convert orientation degrees to quaternion (assuming rotation around Z axis)
            double orientation_rad = orientation_deg * M_PI / 180.0;
            tf2::Quaternion quat;
            quat.setRPY(0, 0, orientation_rad);
            marker.pose.orientation.w = quat.w();
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            
            // Set scale
            marker.scale.x = marker_size_;
            marker.scale.y = marker_size_;
            marker.scale.z = 0.01; // Thin plate
            
            // Set color - use different colors based on ID for easier distinction
            marker.color.a = 0.8;
            marker.color.r = ((id * 40) % 255) / 255.0;
            marker.color.g = ((id * 80) % 255) / 255.0;
            marker.color.b = ((id * 160) % 255) / 255.0;
            
            // Add text label with marker ID
            visualization_msgs::Marker label;
            label.header.frame_id = world_frame_;
            label.header.stamp = ros::Time::now();
            label.ns = "aruco_labels";
            label.id = id;
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            
            label.pose.position = pos;
            label.pose.position.z += 0.1; // Place text slightly above marker
            
            label.text = "ID: " + std::to_string(id);
            
            label.scale.z = 0.1; // Text height
            
            label.color.a = 1.0;
            label.color.r = 1.0;
            label.color.g = 1.0;
            label.color.b = 1.0;
            
            // Add arrows to show marker orientation
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = world_frame_;
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "aruco_arrows";
            arrow.id = id;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            arrow.pose.position = pos;
            arrow.pose.orientation = marker.pose.orientation;
            
            arrow.scale.x = marker_size_ * 1.2;  // Arrow length
            arrow.scale.y = 0.02;                // Arrow width
            arrow.scale.z = 0.02;                // Arrow height
            
            arrow.color.a = 1.0;
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 0.0;
            
            // Add to array
            marker_array.markers.push_back(marker);
            marker_array.markers.push_back(label);
            marker_array.markers.push_back(arrow);
        }
        
        marker_pub_.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "marker_visualizer");
    MarkerVisualizer visualizer;
    ros::spin();
    return 0;
}