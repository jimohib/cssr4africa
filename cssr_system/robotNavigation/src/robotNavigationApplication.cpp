/* robotNavigationApplication.cpp
*
* <detailed functional description>
* The component test the functionality of the actuator of the robot using the ROS interface.
* The test is performed by sending commands to the robot and checking if the robot performs the
* expected action. The test is performed in two modes: sequential and parallel. In the sequential
* mode, the tests are performed one after the other. In the parallel mode, the tests are performed
* simultaneously. 

...
* Libraries
* Standard libraries
- std::string, std::vector, std::thread, std::fstream, std::cout, std::endl, std::cin, std::pow, std::sqrt, std::abs
* ROS libraries
- ros/ros.h, ros/package.h, actionlib/client/simple_action_client.h, control_msgs/FollowJointTrajectoryAction.h, geometry_msgs/Twist.h

...
* Parameters
*
* Command-line Parameters
*
* The location in the world to pay attention to in x, y, z coordinates
...
* Configuration File Parameters

* Key                   |     Value 
* --------------------- |     -------------------
* platform                    robot
* environmentMap              scenarioOneEnvironmentMap.dat
* configurationMap            scenarioOneConfigMap.dat
* pathPlanning                astar
* socialDistance              true
* simulatorTopics             simulatorTopics.dat
* robotTopics                 pepperTopics.dat
* verboseMode                 true

...
* Subscribed Topics and Message Types
*
* None
...
* Published Topics and Message Types
* 
* /pepper_dcm/Head_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper_dcm/RightArm_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftArm_controller/follow_joint_trajectory        trajectory_msgs/JointTrajectory
* /pepper_dcm/RightHand_controller/follow_joint_trajectory      trajectory_msgs/JointTrajectory
* /pepper_dcm/LeftHand_controller/follow_joint_trajectory       trajectory_msgs/JointTrajectory
* /pepper_dcm/Pelvis_controller/follow_joint_trajectory         trajectory_msgs/JointTrajectory
* /pepper_dcm/cmd_moveto                                        geometry_msgs/Twist

* /pepper/Head_controller/follow_joint_trajectory               trajectory_msgs/JointTrajectory
* /pepper/RightArm_controller/follow_joint_trajectory           trajectory_msgs/JointTrajectory
* /pepper/LeftArm_controller/follow_joint_trajectory            trajectory_msgs/JointTrajectory
* /pepper/Pelvis_controller/follow_joint_trajectory             trajectory_msgs/JointTrajectory
* /pepper/cmd_vel                                               geometry_msgs/Twist
...
* Input Data Files
*
* pepperTopics.dat
* simulatorTopics.dat
...
* Output Data Files
*
* None
...
* Configuration Files
*
* robotNavigationConfiguration.ini
...
* Example Instantiation of the Module
*
* rosrun cssr_system robotNavigation
...
*
* Author: Adedayo Akinade, Carnegie Mellon University Africa
* Email: aakinade@andrew.cmu.edu
* Date: September 3, 2024
* Version: v1.0
* Author: Birhanu Shimelis Girma, Carnegie Mellon University Africa
* Email: bgirmash@andrew.cmu.edu
* Date: January 7, 2025
* Version: v1.1
*
*/

#include "robot_navigation/robotNavigationInterface.h"

/* Global variables */

// // Configuration parameters
// std::string implementation_platform;
// std::string environment_map_file;
// std::string configuration_map_file;
// int path_planning_algorithm;
// bool social_distance_mode;
// std::string simulator_topics;
// std::string robot_topics;
// string topics_filename;
// bool verbose_mode;

// // Publisher for the velocity commands


double                x;
double                y;
double                theta;

// Robot pose input which contains the x, y, and theta values of the start location
double                x_start;
double                y_start;
double                theta_start;

// Service request which contains the x, y, and theta values of the goal location
double                x_goal;
double                y_goal;
double                theta_goal;

Mat mapImage;
// Mat mapImageColor;
// Mat mapImageLarge;
Mat configurationSpaceImage;

/* This defines the callback function for the /robotNavigation/set_goal service */
bool set_goal(cssr_system::set_goal::Request  &service_request, cssr_system::set_goal::Response &service_response){
    // Extract request parameters
    x_goal = service_request.goal_x;
    y_goal = service_request.goal_y;
    theta_goal = service_request.goal_theta;    
    theta_goal = radians(theta_goal); // Convert the angle to radians

    x_goal = round_floating_point(x_goal, 1);
    y_goal = round_floating_point(y_goal, 1);

    int navigation_goal_success = 0;                         // Stores the status of the navigation goal
    // Open the Environment and COnfiguration Map

    // Plan the path on the configuration map -- Compute the waypoints

    //Navigate to goal location -- Set odometry to current pose

    // Check robot localization value and compare with goal location -- Return true if they match within some tolerance

    // read_robot_pose_input(robot_pose);       //Replace with subscriber to /robotLocalization/pose topic

    // // Retrieve the robot pose values s the start location
    // x_start = robot_pose[0];
    // x_start = round_floating_point(x_start, 1);
    // y_start = robot_pose[1];
    // y_start = round_floating_point(y_start, 1);
    // theta_start = robot_pose[2];
    // theta_start = radians(theta_start); // Convert the angle to radians
    
    x_start = current_x;
    x_start = round_floating_point(x_start, 1);
    y_start = current_y;
    y_start = round_floating_point(y_start, 1);
    theta_start = current_theta;

    if(x_start > (double) x_map_size/100 || y_start > (double) y_map_size/100){
        ROS_ERROR("Robot pose is outside the map");
        service_response.navigation_goal_success = navigation_goal_success;               
        return true;
    }
    if(x_goal > (double) x_map_size/100 || y_goal > (double) y_map_size/100){
        ROS_ERROR("Goal location is outside the map");
        service_response.navigation_goal_success = navigation_goal_success;               
        return true;
    }

    navigation_goal_success = navigate_to_goal(x_start, y_start, theta_start, x_goal, y_goal, theta_goal, path_planning_algorithm, mapImage, configurationSpaceImage, navigation_velocity_publisher, verbose_mode);

    if(navigation_goal_success == 1){
        //Remove this after robotLOcalization is implemented
        // robot_pose[0] = x_goal;
        // robot_pose[1] = y_goal;
        // robot_pose[2] = degrees(theta_goal);
        robot_pose[0] = current_x;
        robot_pose[1] = current_y;
        robot_pose[2] = degrees(current_theta);
        write_robot_pose_input(robot_pose);     
    }

    service_response.navigation_goal_success = navigation_goal_success;                
   
    // Print the response from the service
    ROS_INFO("Response from /robotNavigation/set_goal service: [%ld]\n", (long int)service_response.navigation_goal_success);
    return true;
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "robotNavigation");
    ros::NodeHandle nh;

    ros::ServiceServer set_goal_service = nh.advertiseService("/robotNavigation/set_goal", set_goal);
    ROS_INFO("Robot Navigation Goal Server Ready\n");

    // Read the configuration file
    int config_file_read = 0;
    config_file_read = read_configuration_file(&implementation_platform, &environment_map_file, &configuration_map_file, &path_planning_algorithm, &social_distance_mode, &simulator_topics, &robot_topics, &topics_filename, &verbose_mode);
    print_configuration(implementation_platform, environment_map_file, configuration_map_file, path_planning_algorithm, social_distance_mode, simulator_topics, robot_topics, topics_filename, verbose_mode);
    
    // Check if the configuration file was read successfully
    if(config_file_read == 1){
        ROS_ERROR("Error reading the configuration file\n");
        return 0;
    }   

    /* Create a publisher object for velocity commands */
    /* ----------------------------------------------- */

    std::string wheels_topic;     // stores the wheels topic
    // Extract the topic for the wheels
    if(extract_topic("Wheels", topics_filename, &wheels_topic)){
        ROS_ERROR("Error extracting the wheels topic\n");
        return 0;
    }
    navigation_velocity_publisher = nh.advertise<geometry_msgs::Twist>(wheels_topic, 10);

    // navigation_pelvis_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/pepper_dcm/Pelvis_controller/command", 1000, true);
    navigation_pelvis_publisher = nh.advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("/joint_angles", 1000, true);

    // Start waist stabilization thread
    // std::thread waist_stabilization_thread(stabilize_waist_continuously);

    // mapImage = imread("/home/cssr4a/workspace/pepper_rob_ws/src/cssr4africa/robot_navigation/data/scenarioOneEnvironmentMap.dat", IMREAD_GRAYSCALE);

    // namedWindow(mapWindowName, WINDOW_AUTOSIZE);// create the window

    // imshow(mapWindowName, mapImage); // Display map image
    // waitKey(10000);

    bool                 debug = true;
   
    FILE                 *fp_in;                    
    char                 path[MAX_FILENAME_LENGTH];
    char                 input_filename[MAX_FILENAME_LENGTH]                 = "moveToInput.txt";
    char                 locomotion_parameter_filename[MAX_FILENAME_LENGTH]  = "parameters230.txt";
    char                 navigation_map_filename[MAX_FILENAME_LENGTH]         = "";
    char                 environment_map_filename[MAX_FILENAME_LENGTH]         = "";
    char                 navigation_pathway_filename[MAX_FILENAME_LENGTH]         = "";
    char                 path_and_input_filename[MAX_FILENAME_LENGTH]        = "";
    int                  end_of_file;
    bool                 success = true;

    double                publish_rate                = 10;   // rate at which cmd_vel commands are published

    /* Create a subscriber object for the odom topic -- replace with /robotLocalization/pose */
    /* --------------------------------------------- */
    
    if (debug) printf("Subscribing to odom\n");
    // ros::Subscriber sub = nh.subscribe("/naoqi_driver/odom", 1, &odomMessageReceived);   
    ros::Subscriber sub = nh.subscribe("/robotLocalization/pose", 1, &poseMessageReceived);
     

    // /* open the input file and read the data */
    // /* ===================================== */


    // /* construct the full path and filename */
    // /* ------------------------------------ */
    
    packagedir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory



    /* get the dimensions of the environment map in centimeters */
    /* -------------------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, environment_map_file.c_str());

    mapImage = imread(path_and_input_filename, IMREAD_GRAYSCALE);


    /* get the dimensions of the navigation map in centimeters */
    /* -------------------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, configuration_map_file.c_str());

    configurationSpaceImage = imread(path_and_input_filename, IMREAD_GRAYSCALE);

    x_map_size = configurationSpaceImage.cols;
    y_map_size = configurationSpaceImage.rows;

    // Get image dimensions (room dimensions based on image size)
    image_width = configurationSpaceImage.cols;
    image_height = configurationSpaceImage.rows;
    // printf("Image width: %d, Image height: %d\n", image_width, image_height);

    room_width = (double) image_width / 100;  // Width of the room in meters
    room_height = (double) image_height / 100;  // Height of the room in meters
    // printf("Room width: %.2f, Room height: %.2f\n", room_width, room_height);

    /* get the locomotion parameter data */
    /* --------------------------------- */

    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, locomotion_parameter_filename);
    
    readLocomotionParameterData(path_and_input_filename, &locomotionParameterData);

    /* convert map image to a graph  */
    /* ----------------------------- */

    build_graph_from_map(configurationSpaceImage, graph, path_planning_algorithm);

    if(locomotionParameterData.robot_available){
        move_robot_actuators_to_default();
    }

    while(ros::ok()){       
        ROS_INFO_THROTTLE(10, "Robot Navigation Node Running...");
        ros::spinOnce(); 

    }

    
    return 0;
}
