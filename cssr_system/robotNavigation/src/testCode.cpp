
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <thread>
#include <atomic>

ros::Publisher cmd_vel_pub;
ros::Publisher joint_angles_pub;
std::atomic<bool> is_moving(false);

void stabilizeWaist()
{
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("WaistPitch");
    msg.joint_angles.push_back(0.0);  // Set waist pitch to 0 (upright position)
    msg.speed = 0.1;  // Adjust speed as needed
    msg.relative = 0;  // Absolute position

    joint_angles_pub.publish(msg);
}

void stabilizeWaistContinuously()
{
    ros::Rate rate(10);  // Adjust the rate as needed
    while (ros::ok())
    {
        if (is_moving)
        {
            stabilizeWaist();
        }
        rate.sleep();
    }
}

void moveRobot(float linear_x, float linear_y, float angular_z)
{
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_x;
    cmd_vel_msg.linear.y = linear_y;
    cmd_vel_msg.angular.z = angular_z;

    cmd_vel_pub.publish(cmd_vel_msg);
    is_moving = (linear_x != 0 || linear_y != 0 || angular_z != 0);

    if (is_moving)
    {
        ROS_INFO("Moving robot and stabilizing waist");
    }
    else
    {
        ROS_INFO("Robot stopped");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pepper_controller");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joint_angles_pub = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1);

    // Start waist stabilization thread
    std::thread waist_stabilization_thread(stabilizeWaistContinuously);

    // Example usage
    ros::Rate loop_rate(1);  // 1 Hz for this example
    while (ros::ok())
    {
        // Move forward
        moveRobot(0.1, 0.0, 0.0);
        ros::Duration(5.0).sleep();  // Move for 5 seconds

        // Stop
        moveRobot(0.0, 0.0, 0.0);
        ros::Duration(2.0).sleep();  // Pause for 2 seconds

        // Turn
        moveRobot(0.0, 0.0, 0.5);
        ros::Duration(3.0).sleep();  // Turn for 3 seconds

        // Stop
        moveRobot(0.0, 0.0, 0.0);
        ros::Duration(2.0).sleep();  // Pause for 2 seconds

        loop_rate.sleep();
    }

    // Clean up
    is_moving = false;
    if (waist_stabilization_thread.joinable())
    {
        waist_stabilization_thread.join();
    }

    return 0;
}