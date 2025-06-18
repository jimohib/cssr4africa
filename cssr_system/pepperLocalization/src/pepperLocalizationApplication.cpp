/* robotLocalizationApplication.cpp
*
* <detailed functional description>
* The component tests the functionality of the localization of the robot using the ROS interface.
* The test is performed by initializing the robot's position and orientation and checking if the
* robot's localization system updates its state correctly. The test is performed in a continuous
* loop, periodically logging the status of the localization system.

...
* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::stod
* ROS libraries
- ros/ros.h, ros/package.h, geometry_msgs/PoseWithCovarianceStamped.h

...
* Parameters
*
* Command-line Parameters
*
* The initial position and orientation of the robot in x, y coordinates and theta (yaw) angle.
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
*platform                       robot
camera                          FrontCamera
resetInterval                   100
robotTopics                     pepperTopics.dat
simulatorTopics                 simulatorTopics.dat
verboseMode                     false

* Configuration Files
*
* robotLocalizationConfiguration.ini
...
* Example Instantiation of the Module
*
* rosrun cssr_system robotLocalization
...
*
* Author: Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email: bgirmash@andrew.cmu.edu
* Date: September 7, 2024
* Version: v1.0
*
*/

#include "robotLocalization/robotLocalizationInterface.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotLocalization");
    ros::NodeHandle nh;

    ROS_INFO("Robot Localization Starting...");

    RobotLocalization rl;

    ROS_INFO("Robot Localization Ready");
    double robot_initial_x = 0.0;
    double robot_initial_y = 0.0;
    double robot_initial_theta = 0.0;

    if (argc > 1)
    {
        robot_initial_x = std::stod(argv[1]);
        robot_initial_y = std::stod(argv[2]);
        robot_initial_theta = std::stod(argv[3]);
    }
    else
    {
        if (!nh.getParam("initial_robot_x", robot_initial_x))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_x', using default value: %.2f", robot_initial_x);
        }
        if (!nh.getParam("initial_robot_y", robot_initial_y))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_y', using default value: %.2f", robot_initial_y);
        }
        if (!nh.getParam("initial_robot_theta", robot_initial_theta))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_theta', using default value: %.2f", robot_initial_theta);
        }
    }

    rl.setInitialValues(robot_initial_x, robot_initial_y, robot_initial_theta);

    while (ros::ok())
    {
        ROS_INFO_THROTTLE(10, "Robot Localization node running...");
        ros::spinOnce();
    }

    if (nh.hasParam("initial_robot_x"))
    {
        nh.deleteParam("initial_robot_x");
    }
    if (nh.hasParam("initial_robot_y"))
    {
        nh.deleteParam("initial_robot_y");
    }
    if (nh.hasParam("initial_robot_theta"))
    {
        nh.deleteParam("initial_robot_theta");
    }

    return 0;
}





// #include "robotLocalization/robotLocalizationInterface.h"

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "robot_localization");
//     RobotLocalization localization;
//     ros::spin();
//     return 0;
// }
