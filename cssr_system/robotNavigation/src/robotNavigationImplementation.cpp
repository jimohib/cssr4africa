

#include "robot_navigation/robotNavigationInterface.h"

// Directory where the package is located
std::string packagedir;

// Coordinates of the robot in the world (x, y, z, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};

// Flag to indicate if a waypoint is being followed
bool waypointFlag;

// Configuration parameters
std::string implementation_platform;
std::string environment_map_file;
std::string configuration_map_file;
int path_planning_algorithm;
bool social_distance_mode;
std::string simulator_topics;
std::string robot_topics;
string topics_filename;
bool verbose_mode;

// Publisher for the velocity commands
ros::Publisher navigation_velocity_publisher;
ros::Publisher navigation_pelvis_publisher;

std::atomic<bool> is_moving(false);  // Flag to indicate if the robot is moving

// Size of the map
int x_map_size;
int y_map_size;

int image_width;
int image_height;

double room_width;  // Width of the room in meters
double room_height;  // Height of the room in meters

// Window names to display the maps
string mapWindowName = "Environment Map"; 

locomotionParameterDataType locomotionParameterData;

geometry_msgs::Twist msg;

std::vector<double> leg_home_position = {0.0, 0.0, 0.0};  // Hip pitch, hip roll, knee pitch
std::vector<double> head_home_position = {0.0, 0.0};   // Head pitch and yaw
// Mat images to display the maps
// Mat mapImage;
// Mat mapImageColor;
// Mat mapImageLarge;
// Mat configurationSpaceImage;

std::vector<std::vector<int>> graph;  // Graph to store the map
std::vector<int> robot_path; // Path to store the path
std::vector<pointType> valid_path;
std::vector<waypointType> valid_waypoints;

// 4-way movement (BFS)
int directions_4_way[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// 8-way movement (Dijkstra, A*)
int directions_8_way[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};


/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;   // used to turn debug message on

    std::string data_file = "robotPose.dat";    // data filename
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    std::string x_key = "x";                                         // x key
    std::string y_key = "y";                                         // y key
    std::string theta_key = "theta";                                 // theta key

    std::string x_value;                                             // x value
    std::string y_value;                                             // y value
    std::string z_value;                                             // z value
    std::string theta_value;                                         // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        ROS_ERROR("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the data file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        // Read the x value of the robot pose
        if (param_key == x_key){
            x_value = param_value;
            robot_pose_input[0] = std::stod(param_value);
        }

        // Read the y value of the robot pose
        else if (param_key == y_key){
            y_value = param_value;
            robot_pose_input[1] = std::stod(param_value);
        }

        // Read the theta value of the robot pose
        else if (param_key == theta_key){
            theta_value = param_value;
            robot_pose_input[2] = std::stod(param_value);
        }
    }
    // Close the data file
    data_if.close();

    if (debug_mode){
        printf("Robot location input:\n");
        printf("\tX: %.2f\n", robot_pose_input[0]);
        printf("\tY: %.2f\n", robot_pose_input[1]);
        printf("\tTheta: %.2f\n", robot_pose_input[2]);
    }
}

/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void write_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;   // used to turn debug message on

    std::string data_file = "robotPose.dat";    // data filename
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    std::string x_key = "x";                                         // x key
    std::string y_key = "y";                                         // y key
    std::string theta_key = "theta";                                 // theta key

    std::string x_value;                                             // x value
    std::string y_value;                                             // y value
    std::string z_value;                                             // z value
    std::string theta_value;                                         // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/robotNavigation/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open data file for writing
      std::ofstream data_of(data_path_and_file.c_str());
      if (!data_of.is_open()){
         printf("Unable to open the data file %s\n", data_path_and_file.c_str());
         prompt_and_exit(1);
      }

      // Write the robot pose to the data file
      data_of << x_key << "\t\t\t" << robot_pose_input[0] << std::endl;
      data_of << y_key << "\t\t\t" << robot_pose_input[1] << std::endl;
      data_of << theta_key << "\t\t" << robot_pose_input[2] << std::endl;

      // Close the data file
      data_of.close();
}

/*  
 *   Function to extract the topic from the topics file
 *   The function reads the topics file and extracts the topic for the specified key.
 *
 *   @param:
 *       key: the key to search for in the topics file
 *       topic_file_name: the topics filename
 *       topic_name: the topic name extracted
 *
 *   @return:
 *       0 if successful, 1 otherwise
 */
int extract_topic(string key, string topic_file_name, string *topic_name){
    bool debug = false;   // used to turn debug message on
    
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/robotNavigation/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) ROS_INFO("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topic_line_read)){
        std::istringstream iss(topic_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        if (param_key == key) {                     // if the key is found
            topic_value = param_value;              // set the topic value
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        ROS_ERROR("Unable to find a valid topic for actuator: %s. Please check the topics file.\n", key.c_str());
        return 1;
    }

    *topic_name = topic_value;                      // set the topic name
    return 0;
}

/*  
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the hand
    ControlClientPtr hand_client;

    // Goal messages for the hand
    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;
    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration)); // Wait for the actuator to reach the specified position

    return;
}


int read_configuration_file(string* platform, string* environment_map_file, string* configuration_map_file, int* path_planning_algorithm, bool* social_distance_mode, string* simulator_topics, string* robot_topics, string* topics_filename, bool* debug_mode){
    std::string config_file = "robotNavigationConfiguration.ini";       // data filename
    std::string config_path;                                            // data path
    std::string config_path_and_file;                                   // data path and filename
     
    std::string platform_key = "platform";                              // platform key 
    std::string environment_map_file_key = "environmentMap";            // camera key
    std::string configuration_map_file_key = "configurationMap";        // realignment threshold key
    std::string path_planning_algorithm_key = "pathPlanning";           // realignment threshold key
    std::string social_distance_mode_key = "socialDistance";          // x offset to head yaw key
    std::string simulator_topics_key = "simulatorTopics";               // simulator topics key
    std::string robot_topics_key = "robotTopics";                       // robot topics key
    std::string verbose_mode_key = "verboseMode";                       // verbose mode key

    std::string platform_value;                                         // platform value 
    std::string environment_map_file_value;                             // camera value
    std::string configuration_map_file_value;                           // realignment threshold value
    std::string path_planning_algorithm_value;                            // realignment threshold value
    std::string social_distance_mode_value;                             // x offset to head yaw value
    std::string simulator_topics_value;                                 // simulator topics value
    std::string robot_topics_value;                                     // robot topics value
    std::string verbose_mode_value;                                     // verbose mode value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += "/robotNavigation/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;

    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        return 1;
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key, param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (param_key == platform_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            platform_value = param_value;
            *platform = param_value;
            if(platform_value != "robot" && platform_value != "simulator"){
                printf("Platform value not supported. Supported values are: robot and simulator\n");
                return 1;
            }
        }
        
        else if (param_key == environment_map_file_key){ 
            environment_map_file_value = param_value;
            *environment_map_file = param_value;
        }

        else if (param_key == configuration_map_file_key){ 
            configuration_map_file_value = param_value;
            *configuration_map_file = param_value;
        }

        else if (param_key == path_planning_algorithm_key){ 
            path_planning_algorithm_value = param_value;
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            if(param_value == "bfs"){
                *path_planning_algorithm = BFS_ALGORITHM;
            }
            else if((param_value == "dijkstra" || param_value == "dijsktra")){
                *path_planning_algorithm = DIJKSTRA_ALGORITHM;
            }
            else if((param_value == "a*") || (param_value == "a-star") || (param_value == "a_star") || (param_value == "astar")){
                *path_planning_algorithm = ASTAR_ALGORITHM;
            }
            else{
                printf("Path planning algorithm value not supported. Supported values are: bfs, dijkstra, and a*\n");
                return 1;
            }
        }

        else if (param_key == social_distance_mode_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            social_distance_mode_value = param_value;
            if(social_distance_mode_value == "true"){
                *social_distance_mode = true;
            }
            else if(social_distance_mode_value == "false"){
                *social_distance_mode = false;
            }
            else{
                printf("Social distance mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }

        else if (param_key == simulator_topics_key){ 
            simulator_topics_value = param_value;
            *simulator_topics = param_value;
        }

        else if (param_key == robot_topics_key){ 
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }

        else if (param_key == verbose_mode_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            verbose_mode_value = param_value;
            if(verbose_mode_value == "true"){
                *debug_mode = true;
            }
            else if(verbose_mode_value == "false"){
                *debug_mode = false;
            }
            else{
                printf("Verbose mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }
    }
    data_if.close();

    if(*platform == "" || *environment_map_file == "" || *configuration_map_file == "" || *simulator_topics == "" || *robot_topics == ""){
        printf("Unable to find a valid configuration. Verify you have values in the configuration.\n");
        return 1;
    }

    if (platform_value == "robot"){
        *topics_filename = *robot_topics;
    }
    else if(platform_value == "simulator"){
        *topics_filename = *simulator_topics;
    }

    return 0;
}

/* Print the robot navigation configuration */
void print_configuration(string platform, string environment_map_file, string configuration_map_file, int path_planning_algorithm, bool social_distance_mode, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode){
    printf("Platform: %s\n", platform.c_str());
    printf("Environment Map File: %s\n", environment_map_file.c_str());
    printf("Configuration Map File: %s\n", configuration_map_file.c_str());
    printf("Path Planning Algorithm: %s\n", path_planning_algorithm == BFS_ALGORITHM ? "BFS" : path_planning_algorithm == DIJKSTRA_ALGORITHM ? "Dijsktra" : "A*");
    printf("Social Distance Mode: %s\n", social_distance_mode ? "true" : "false");
    printf("Simulator Topics: %s\n", simulator_topics.c_str());
    printf("Robot Topics: %s\n", robot_topics.c_str());
    printf("Topics Filename: %s\n", topics_filename.c_str());
    printf("Debug Mode: %s\n", debug_mode ? "true" : "false");
}

/******************************************************************************

global variables with the current robot pose

*******************************************************************************/

/* global variables with the initial and current robot pose, the odometry pose, and the difference between the initial pose and the initial odometry pose  */

double                  initial_x        = 0; 
double                  initial_y        = 0;
double                  initial_theta    = 0;
double                  current_x        = 0; 
double                  current_y        = 0; 
double                  current_theta    = 0;
double                  odom_x           = 0;
double                  odom_y           = 0;
double                  odom_theta       = 0;
double                  adjustment_x     = 0; 
double                  adjustment_y     = 0;
double                  adjustment_theta = 0;

/******************************************************************************

odomMessageReceived

Callback function, executed each time a new pose message arrives 

*******************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry &msg)
{
   bool debug = true;

   double x, y;

   odom_x = msg.pose.pose.position.x;
   odom_y = msg.pose.pose.position.y;
   odom_theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

   /* change frame of reference from arbitrary odometry frame of reference to the world frame of reference */

   /* translation of origin */

   x = odom_x + adjustment_x - initial_x;
   y = odom_y + adjustment_y - initial_y;

   /* rotation about origin */

   current_x = x * cos(adjustment_theta) + y * -sin(adjustment_theta);
   current_y = x * sin(adjustment_theta) + y * cos(adjustment_theta);

   current_x += initial_x;
   current_y += initial_y;

   current_theta = odom_theta + adjustment_theta;

   /* check to ensure theta is still in the range -PI to +PI */

   if (current_theta < -PI)
      current_theta += 2 * PI;
   else if (current_theta > PI)
      current_theta -= 2 * PI;

   // printf("odom_x,y,theta %5.3f %5.3f %5.3f; adjustment_x,y,theta  %5.3f %5.3f %5.3f; x, y %5.3f %5.3f; current_x,y,theta %5.3f %5.3f %5.3f\n",  odom_x, odom_y, odom_theta, adjustment_x, adjustment_y, adjustment_theta, x, y, current_x, current_y, current_theta);

   if (debug)
   {
      // printf("Odometry: position = (%5.3f, %5.3f) orientation = %5.3f\n", current_x, current_y, current_theta);
      ROS_INFO_THROTTLE(1, "Odometry: position = (%5.3f, %5.3f) orientation = %5.3f", current_x, current_y, current_theta);

   }
   
}

void poseMessageReceived(const geometry_msgs::Pose2D &msg)
{
    bool debug = true;

    current_x = msg.x;
    current_y = msg.y;
    current_theta = radians(msg.theta);
    // current_theta = fmod(current_theta, 2 * PI);
   
}


/*******************************************************************************

readLocomotionParameterData

Read locomotion parameters from file     

*******************************************************************************/

void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData)
{

   bool debug = true;
   int i;
   int j;
   int k;

   keyword keylist[NUMBER_OF_KEYS] = {
       "position_tolerance",
       "position_tolerance_goal",
       "angle_tolerance_orienting",
       "angle_tolerance_going",
       "position_gain_dq",
       "angle_gain_dq",
       "position_gain_mimo",
       "angle_gain_mimo",
       "min_linear_velocity",
       "max_linear_velocity",
       "min_angular_velocity",
       "max_angular_velocity",
       "clearance",
       "shortest_path_algorithm",
       "robot_available"};

   keyword key;   // the key string when reading parameters
   keyword value; // the value string, used for the SHORTEST_PATH_ALGORITHM key
   keyword robot_state; // the robot_state string, used for the ROBOT_AVAILABLE key

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
     printf("Error can't open locomition parameter file %s\n", filename);
     prompt_and_exit(0);
   }

   /*** set default values ***/

   locomotionParameterData->position_tolerance = 0.025;
   locomotionParameterData->position_tolerance_goal = 0.01;
   locomotionParameterData->angle_tolerance_orienting = 0.075;
   locomotionParameterData->angle_tolerance_going = 0.075;
   locomotionParameterData->position_gain_dq = 0.3;
   locomotionParameterData->angle_gain_dq = 0.3;
   locomotionParameterData->position_gain_mimo = 0.2;
   locomotionParameterData->angle_gain_mimo = 0.5;
   locomotionParameterData->min_linear_velocity = 0.015;
   locomotionParameterData->max_linear_velocity = 0.5;
   locomotionParameterData->min_angular_velocity = 0.09;
   locomotionParameterData->max_angular_velocity = 1.0;
   locomotionParameterData->clearance = 0.05;
   locomotionParameterData->shortest_path_algorithm = ASTAR; // BFS, DIJKSTRA, ASTAR
   locomotionParameterData->robot_available = FALSE; // TRUE/FALSE

   /*** get the key-value pairs ***/

   for (i = 0; i < NUMBER_OF_KEYS; i++)
   {

     fgets(input_string, STRING_LENGTH, fp_config);
     // if (debug)  printf ("Input string: %s",input_string);

     /* extract the key */

     sscanf(input_string, " %s", key);

     for (j = 0; j < (int)strlen(key); j++)
        key[j] = tolower(key[j]);

     // if (debug)  printf ("key: %s\n",key);

     for (j = 0; j < NUMBER_OF_KEYS; j++)
     {
        if (strcmp(key, keylist[j]) == 0)
        {
           switch (j)
           {
           case 0:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance)); // position_tolerance
              break;
           case 1:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_tolerance_goal)); // angle_tolerance_orienting
              break;
           case 2:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_orienting)); // angle_tolerance_orienting
              break;
           case 3:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_tolerance_going)); // angle_tolerance_going
              break;
           case 4:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_dq)); // position_gain_dq)
              break;
           case 5:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_dq)); // angle_gain_dq
              break;
           case 6:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->position_gain_mimo)); // position_gain_mimo
              break;
           case 7:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->angle_gain_mimo)); // angle_gain_mimo
              break;
           case 8:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_linear_velocity)); // min_linear_velocity
              break;
           case 9:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_linear_velocity)); // max_linear_velocity
              break;
           case 10:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->min_angular_velocity)); // min_angular_velocity
              break;
           case 11:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->max_angular_velocity)); // max_angular_velocity
              break;
           case 12:
              sscanf(input_string, " %s %lf", key, &(locomotionParameterData->clearance)); // required clearance between robot and obstacle
              break;
           case 13:
              sscanf(input_string, " %s %s", key, value); // BFS,  Dijkstra,  ASTAR
              for (k = 0; k < (int)strlen(value); k++)
                 value[k] = tolower(value[k]);
              if (strcmp(value, "bfs") == 0)
                 locomotionParameterData->shortest_path_algorithm = BFS;
              else if (strcmp(value, "dijkstra") == 0)
                 locomotionParameterData->shortest_path_algorithm = DIJKSTRA;
              else if (strcmp(value, "astar") == 0)
                 locomotionParameterData->shortest_path_algorithm = ASTAR;
              else
                 locomotionParameterData->shortest_path_algorithm = ASTAR; // default value is A-star
              break;
           case 14:
              sscanf(input_string, " %s %s", key, robot_state); // true/false
              for (k = 0; k < (int)strlen(robot_state); k++)
                 robot_state[k] = tolower(robot_state[k]);
              if (strcmp(robot_state, "true") == 0)
                 locomotionParameterData->robot_available = TRUE;
              else if (strcmp(robot_state, "false") == 0)
                 locomotionParameterData->robot_available = FALSE;
           }
        }
     }
   }

   // printf("value %s\n", value);
   if (debug)
   {
     printf("POSITION_TOLERANCE:        %.2f\n", locomotionParameterData->position_tolerance);
     printf("POSITION_TOLERANCE_GOAL:   %.2f\n", locomotionParameterData->position_tolerance_goal);
     printf("ANGLE_TOLERANCE_ORIENTING: %.2f\n", locomotionParameterData->angle_tolerance_orienting);
     printf("ANGLE_TOLERANCE_GOING:     %.2f\n", locomotionParameterData->angle_tolerance_going);
     printf("POSITION_GAIN_DQ:          %.2f\n", locomotionParameterData->position_gain_dq);
     printf("ANGLE_GAIN_DQ:             %.2f\n", locomotionParameterData->angle_gain_dq);
     printf("POSITION_GAIN_MIMO:        %.2f\n", locomotionParameterData->position_gain_mimo);
     printf("ANGLE_GAIN_MIMO:           %.2f\n", locomotionParameterData->angle_gain_mimo);
     printf("MIN_LINEAR_VELOCITY:       %.2f\n", locomotionParameterData->min_linear_velocity);
     printf("MAX_LINEAR_VELOCITY:       %.2f\n", locomotionParameterData->max_linear_velocity);
     printf("MIN_ANGULAR_VELOCITY:      %.2f\n", locomotionParameterData->min_angular_velocity);
     printf("MAX_ANGULAR_VELOCITY:      %.2f\n", locomotionParameterData->max_angular_velocity);
     printf("CLEARANCE:                 %.2f\n", locomotionParameterData->clearance);
     printf("SHORTEST_PATH_ALGORITHM    %d\n", locomotionParameterData->shortest_path_algorithm);
     printf("ROBOT_AVAILABLE            %d\n", locomotionParameterData->robot_available);
   }
}

/***************************************************************************************************************************

   Definitions for paths and waypoints 

****************************************************************************************************************************/


// Function to build the graph from the image
void build_graph_from_map(cv::Mat& img, std::vector<std::vector<int>>& graph, int algorithm) {
    int rows = img.rows;
    int cols = img.cols;

    int (*directions)[2];
    int num_directions;

    if (algorithm == BFS) {
        directions = directions_4_way;
        num_directions = 4;
    } else { // DIJKSTRA and ASTAR use 8-way movement
        directions = directions_8_way;
        num_directions = 8;
    }

    graph.resize(rows * cols);

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (img.at<uchar>(y, x) == 255) {  // Free space
                int current_node = y * cols + x;
                for (int i = 0; i < num_directions; ++i) {
                    int new_x = x + directions[i][0];
                    int new_y = y + directions[i][1];
                    if (new_x >= 0 && new_x < cols && new_y >= 0 && new_y < rows) {
                        if (img.at<uchar>(new_y, new_x) == 255) {  // Neighbor is free space
                            int neighbor_node = new_y * cols + new_x;
                            graph[current_node].push_back(neighbor_node);
                        }
                    }
                }
            }
        }
    }
}


// Manhattan distance heuristic function
int manhattan_distance_heuristic(int start, int node, int goal, int cols) {
    int goal_x = goal % cols;
    int goal_y = goal / cols;
    int node_x = node % cols;
    int node_y = node / cols;

    int heuristic = 0;
    heuristic = std::abs(goal_x - node_x) + std::abs(goal_y - node_y);

    double D = 6;
    double D2 = D * 1.414;

    int dx = std::abs(node_x - goal_x);
    int dy = std::abs(node_y - goal_y);

    return heuristic;
}

// Euclidean distance heuristic function
int euclidean_distance_heuristic(int start, int node, int goal, int cols) {
    int goal_x = goal % cols;
    int goal_y = goal / cols;
    int node_x = node % cols;
    int node_y = node / cols;
    int heuristic = static_cast<int>(std::sqrt(std::pow(goal_x - node_x, 2) + std::pow(goal_y - node_y, 2)));
   //  heuristic = heuristic * 1000;

    int start_x = start % cols;
    int start_y = start / cols;

      int dx_start = start_x - goal_x;
      int dy_start = start_y - goal_y;

      int dx_node = node_x - goal_x;
      int dy_node = node_y - goal_y;

      int cross_product = abs((dx_start * dy_node )- (dy_start * dx_node));

      heuristic += cross_product * 0.001;
    return heuristic;
}


void convert_world_to_pixel(double world_x, double world_y, int& pixel_x, int& pixel_y, double room_width, double room_height, int image_width, int image_height) {
    pixel_x = static_cast<int>(world_x * image_width / room_width);
    pixel_y = static_cast<int>(image_height - (world_y * image_height / room_height));
}

void convert_pixel_to_world(int pixel_x, int pixel_y, double& world_x, double& world_y, double room_width, double room_height, int image_width, int image_height) {
    world_x = static_cast<double>(pixel_x) * (room_width / image_width);
    world_y = room_height - static_cast<double>(pixel_y) * (room_height / image_height); // Invert Y for world coordinates
}

void draw_path_on_map(const std::vector<waypointType>& valid_waypoints, const cv::Mat& img, const std::string& output_path) {
    cv::Mat output_image;

    cvtColor(img, output_image, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < valid_waypoints.size(); i++) {
        // Draw crosshair
        cv::drawMarker(output_image, cv::Point(valid_waypoints[i].x, valid_waypoints[i].y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    }

   for (int i=0; i<valid_path.size(); i++) {
      cv::drawMarker(output_image, cv::Point(valid_path[i].x, valid_path[i].y), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 5, 1);
   }

    cv::imwrite(output_path, output_image); // Save the output image
}

double path_orientation(std::vector<pointType>& valid_path, int i) {
    double delta_x;
    double delta_y;
    double orientation = 0;

    if (valid_path.size() < 3) {
        orientation = 0;
    } else if (i == 0) {  // Special case: first point
        delta_x = valid_path[i + 1].x - valid_path[i].x;
        delta_y = valid_path[i + 1].y - valid_path[i].y;
    } else if (i == valid_path.size() - 1) {  // Special case: last point
        delta_x = valid_path[i].x - valid_path[i - 1].x;
        delta_y = valid_path[i].y - valid_path[i - 1].y;
    } else {
        delta_x = valid_path[i + 1].x - valid_path[i - 1].x;
        delta_y = valid_path[i + 1].y - valid_path[i - 1].y;
    }

    orientation = std::atan2(-delta_y, delta_x);  // Compute orientation in map frame of reference, not image frame of reference

    return orientation;
}

void compute_waypoints(std::vector<int>& robot_path, std::vector<pointType>& valid_path, std::vector<waypointType>& valid_waypoints, int suppression_factor, double curvature_angle){
    double valid_orientation;


   // Convert the robot path to valid path
    for(int i = 0; i < robot_path.size(); i++){
      int x = robot_path[i] % image_width;
      int y = robot_path[i] / image_width;
      valid_path.push_back({x, y});
   }

    valid_orientation = path_orientation(valid_path, 0);
    valid_waypoints.push_back({valid_path[0].x, valid_path[0].y, valid_orientation});

    int i;
    for (i = 1; i < valid_path.size() - 1; i++){
        double next_orientation = path_orientation(valid_path, i+1);
        double previous_orientation = path_orientation(valid_path, i-1);
        if(fabs(next_orientation - previous_orientation) > curvature_angle){
            valid_orientation = path_orientation(valid_path, i);
            valid_waypoints.push_back({valid_path[i].x, valid_path[i].y, valid_orientation});
        }
    }

    if(i < valid_path.size()){
        valid_orientation = path_orientation(valid_path, i);
        valid_waypoints.push_back({valid_path[i].x, valid_path[i].y, valid_orientation});
    }

    

    int j;
    for(int i = 0; i < valid_waypoints.size()-1; i++){
        j = i+1;
        while(j < valid_waypoints.size() && sqrt(pow(valid_waypoints[i].x - valid_waypoints[j].x, 2) + pow(valid_waypoints[i].y - valid_waypoints[j].y, 2)) < suppression_factor){
            if(j < valid_waypoints.size() - 1){
                for(int k = j; k < valid_waypoints.size() - 1; k++){
                    valid_waypoints[k].x = valid_waypoints[k + 1].x;
                    valid_waypoints[k].y = valid_waypoints[k + 1].y;
                    valid_waypoints[k].theta = valid_waypoints[k + 1].theta;
                }
            }
            valid_waypoints.pop_back();
        }
    }

    // Print waypoints
    // print_waypoints(valid_waypoints, room_width, room_height, image_width, image_height);
}


void print_waypoints(std::vector<waypointType>& valid_waypoints, double room_width, double room_height, int image_width, int image_height){
    for(int i = 0; i < valid_waypoints.size(); i++){
        // Convert to world coordinates
        double world_x;
        double world_y;
        convert_pixel_to_world(valid_waypoints[i].x, valid_waypoints[i].y, world_x, world_y, room_width, room_height, image_width, image_height);
      //   double world_x = (valid_waypoints[i].x) * (room_width / image_width);
        // double world_y = room_height - (valid_waypoints[i].y) * (room_height / image_height); // Invert Y for world coordinates

        // ROS_INFO("Path Point: (%.2f, %.2f)", world_x, world_y);
        ROS_INFO("Waypoint %d: x = %.2f, y = %.2f, theta = %.2f\n", i, world_x, world_y, valid_waypoints[i].theta);
    }
}

// A* pathfinding algorithm
std::vector<int> astar(int start, int goal, const std::vector<std::vector<int>>& graph, int cols, int heuristic_type) {
    auto heuristic = (heuristic_type == MANHATTAN_HEURISTIC) ? manhattan_distance_heuristic : euclidean_distance_heuristic;

    std::vector<int> dist(graph.size(), INT_MAX);
    std::vector<int> came_from(graph.size(), -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0 + heuristic(start, start, goal, cols), start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            int new_dist = dist[current] + 1;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                came_from[neighbor] = current;
                int priority = new_dist + heuristic(current, neighbor, goal, cols);
                pq.push({priority, neighbor});
            }
        }
    }

    return {};
}


std::vector<int> bfs(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::queue<int> queue;
    std::vector<int> came_from(graph.size(), -1);
    std::vector<bool> visited(graph.size(), false);

    queue.push(start);
    visited[start] = true;

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                came_from[neighbor] = current;
                queue.push(neighbor);
            }
        }
    }
    return {};
}

std::vector<int> dijkstra(int start, int goal, const std::vector<std::vector<int>>& graph) {
    std::vector<int> dist(graph.size(), INT_MAX);
    std::vector<int> came_from(graph.size(), -1);
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    dist[start] = 0;
    pq.push({0, start});

    while (!pq.empty()) {
        int current = pq.top().second;
        pq.pop();

        if (current == goal) {
            std::vector<int> path;
            while (current != -1) {
                path.push_back(current);
                current = came_from[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int neighbor : graph[current]) {
            int new_dist = dist[current] + 1;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                came_from[neighbor] = current;
                pq.push({new_dist, neighbor});
            }
        }
    }
    return {};
}

/***************************************************************************************************************************

   General purpose function definitions 

****************************************************************************************************************************/

/* return the sign of a number as +/- 1 */

int signnum(double x)
{
    if (x >= 0.0){
        return 1;
    }
    return -1;
}

void display_error_and_exit(char error_message[])
{
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void print_message_to_file(FILE *fp, char message[])
{
   fprintf(fp, "The message is: %s\n", message);
}

#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif

std::vector<cv::Point> calculateBezierCurve(const std::vector<cv::Point>& points, int numPoints) {
    std::vector<cv::Point> curvePoints;
    int n = points.size() - 1;
    
    for (double t = 0; t <= 1; t += 1.0 / numPoints) {
        double x = 0, y = 0;
        for (int i = 0; i <= n; i++) {
            double coefficient = std::pow(1 - t, n - i) * std::pow(t, i) * 
                                 std::tgamma(n + 1) / (std::tgamma(i + 1) * std::tgamma(n - i + 1));
            x += coefficient * points[i].x;
            y += coefficient * points[i].y;
        }
        curvePoints.push_back(cv::Point(static_cast<int>(x), static_cast<int>(y)));
    }
    
    return curvePoints;
}

int plan_robot_path(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int path_planning_algorithm, Mat mapImage, Mat configurationSpaceImage, bool debug){
   // Vertex number of the start and goal cells
    int robot;                              // vertex number of start cell
    int goal;                               // vertex number of goal cell

    // Map values of the start location
    int x_start_map;
    int y_start_map;

    // Map values of the goal location
    int x_goal_map;
    int y_goal_map;

    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    // Waypoint values
    double x_waypoint;
    double y_waypoint;
    double theta_waypoint;

    // Convert the start location to map coordinates
    x_start_map = y_map_size - (int) (start_y * 100);  // NB: convert to cm and change frame of reference
    y_start_map = (int) (start_x * 100);               // ibid.

    convert_world_to_pixel(start_x, start_y, start_pixel_x, start_pixel_y, room_width, room_height, image_width, image_height);
    convert_world_to_pixel(goal_x, goal_y, end_pixel_x, end_pixel_y, room_width, room_height, image_width, image_height);


    // Convert the goal location to map coordinates
    x_goal_map = y_map_size - (int) (goal_y * 100);   // NB: convert to cm and change frame of reference
    y_goal_map = (int) (goal_x * 100);                //


   //  // Obtain the vertex number of the start and goal cells
   //  robot = vertex_number(x_start_map, y_start_map, x_map_size);
   //  goal  = vertex_number(x_goal_map,  y_goal_map,  x_map_size);

    robot = start_pixel_y * image_width + start_pixel_x;
    goal = end_pixel_y * image_width + end_pixel_x;

    printf("Robot vertex: %d, Goal vertex: %d\n", robot, goal);

    int heuristic_choice = MANHATTAN_HEURISTIC;  // Choose heuristic for A* (Manhattan or Euclidean)
   // int heuristic_choice = EUCLIDEAN_HEURISTIC;  // Choose heuristic for A* (Manhattan or Euclidean)
    
    robot_path.clear();
    valid_path.clear();
    valid_waypoints.clear();
   
    if (path_planning_algorithm == BFS) {
        robot_path = bfs(robot, goal, graph);
    } else if (path_planning_algorithm == DIJKSTRA) {
        robot_path = dijkstra(robot, goal, graph);
    } else if (path_planning_algorithm == ASTAR) {
        robot_path = astar(robot, goal, graph, image_width, heuristic_choice);
    }
    /* get the waypoints  */
    /* ------------------ */

    if(robot_path.empty()){
        ROS_ERROR("No path found from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", start_x, start_y, start_theta, goal_x, goal_y, goal_theta);
        return 0;
    }  
    compute_waypoints(robot_path, valid_path, valid_waypoints, SUPPRESSION_FACTOR, CURVATURE_THRESHOLD);

   // mark_waypoints_on_config_map(path_planning_algorithm, mapImage, configurationSpaceImage, debug);
   std::string output_filename = "environmentMapWaypoints";
   mark_waypoints_on_map(path_planning_algorithm, mapImage, output_filename, debug);

   output_filename = "configMapWaypoints";
   mark_waypoints_on_map(path_planning_algorithm, configurationSpaceImage, output_filename, debug);

   return 1;              // Return 1 if the path is found
}

void mark_waypoints_on_map(int path_planning_algorithm, Mat mapImage, std::string output_filename, bool debug){
   // parameters for image write
   vector<int> compressionParams;  

   // Map image in colour
   Mat mapImageColor;

   // Map image in large format
   Mat mapImageLarge;

   // scale the map and configuration images by this factor before displaying
   double image_display_scale_factor  = 4.0;

   /* Draw path in colour with waypoint and display map and configuration space */
   /* ------------------------------------------------------------------------- */
   
   cvtColor(mapImage,mapImageColor,COLOR_GRAY2BGR);

   /* Draw a grid on the output map imitating the size of a tile in the laboratory. This can be commented out */
   int dist=60;

   for(int i=0;i<mapImageColor.rows;i+=dist)
      line(mapImageColor,Point(0,i),Point(mapImageColor.cols,i),Scalar(0,0,0));

   for(int i=0;i<mapImageColor.cols;i+=dist)
      line(mapImageColor,Point(i,0),Point(i,mapImageColor.rows),Scalar(0,0,0));

   /* Mark the waypoints on the map */

   for (int i = 0; i < valid_waypoints.size(); i++) {
        // Draw crosshair
        cv::drawMarker(mapImageColor, cv::Point(valid_waypoints[i].x, valid_waypoints[i].y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 10, 2);
    }

   for (int i=0; i<valid_path.size(); i++) {
      cv::drawMarker(mapImageColor, cv::Point(valid_path[i].x, valid_path[i].y), cv::Scalar(0, 0, 255), cv::MARKER_SQUARE, 1, 1);
   }

   resize(mapImageColor, mapImageLarge, Size(mapImage.cols * image_display_scale_factor, mapImage.rows * image_display_scale_factor), INTER_NEAREST);

   compressionParams.push_back(IMWRITE_PNG_COMPRESSION);
   compressionParams.push_back(9);                                  // 9 implies maximum compression   

   std::string name_extension = "Astar";

   if(path_planning_algorithm == BFS_ALGORITHM){
      name_extension = "BFS";

   }else if(path_planning_algorithm == DIJKSTRA_ALGORITHM){
      name_extension = "Dijkstra";
   }
   else if(path_planning_algorithm == ASTAR_ALGORITHM){
      name_extension = "Astar";
   }

   std::string filename = output_filename + name_extension + ".png";

   save_waypoint_map(compressionParams, mapImageLarge, filename); // Save the map with the path and waypoints
}

// int navigate_to_goal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int path_planning_algorithm, Mat mapImage, ros::Publisher velocity_publisher, bool debug){
int navigate_to_goal(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, int path_planning_algorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug){
   // Set the publish rate for the velocity commands
   ros::Rate rate(PUBLISH_RATE); // Publish  at this rate (in Hz)  until the node is shut down


   int path_found = 0;
   int robot_moved = 0;
   path_found = plan_robot_path(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, path_planning_algorithm, mapImage, configurationSpaceImage, debug);
   if (path_found){
      if(debug){
         printf("Path found from (%5.3f %5.3f %5.3f) to (%5.3f %5.3f %5.3f)\n", start_x, start_y, degrees(start_theta), goal_x, goal_y, degrees(goal_theta));
         // print_waypoints(valid_waypoints, room_width, room_height, image_width, image_height);
      }
      robot_moved = move_robot(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, velocity_publisher, rate, debug);
   }
   return robot_moved;
}

/*  
 *   Function to create a control client
 *   The function creates a control client for the specified topic.
 *
 *   @param:
 *       topic_name: the topic name
 *
 *   @return:
 *       the control client
 */
ControlClientPtr create_client(const std::string& topic_name) {
    // Create a new action client
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 10;                                            // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;                                        // return the action client if the server is available
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    // throw std::runtime_error("Error creating action client for " + topic_name + " controller: Server not available");
    return nullptr;                                                     // return nullptr if the server is not available
}



/* 
 *   Function to return all joints of an actuator to the home position.
 *   This function is called on only one actuator at a time.
 *   It used the global variable set above for the home positions of the actuators.
 *
 * @param:
 *   actuator: string indicating the actuator to move to the home position
 *   topics_filename: string indicating the topics filename
 *   interpolation: integer indicating the interpolation type
 *   debug: boolean indicating the debug mode
 *
 * @return:
 *   None
 */
int go_to_home(std::string actuator, std::string topics_filename, bool debug){
    // ros::Duration(0.5).sleep(); // Wait for one second to ensure that the joint states are updated
    std::vector<double> actuator_state;                             // stores the current state of the actuator joints
    std::vector<double> actuator_home_position;                     // stores the home position of the actuator joints
    ControlClientPtr actuator_client;                               // action client to control the actuator joints
    std::vector<std::string> actuator_joint_names;                  // stores the joint names of the actuator joints
    std::string actuator_topic;                                     // stores the topic of the actuator for control
    int number_of_joints;                                           // stores the number of joints of the actuator
    double home_duration;                                           // stores the duration to move to the home position

    // Set the open hand flag and the hand topic. ust for default, the hand is not to be open in home position
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Extract the actuator topic
    if(extract_topic(actuator, topics_filename, &actuator_topic)){
        return 0;
    }

    // Set the home duration
    home_duration = 0.5;
    if(actuator == "Leg"){                         // Leg
      //   actuator_state = leg_joint_states;
        actuator_home_position = leg_home_position;
        actuator_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }
    else if(actuator == "Head"){                        // Head
      //   actuator_state = head_joint_states;
        actuator_home_position = head_home_position;
        actuator_joint_names = {"HeadPitch", "HeadYaw"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }

    // Vectors to store the trajectory information (positions, velocities, accelerations and durations)
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

   //  // Compute the trajectory to get to the home position
   //  compute_trajectory(actuator_state, actuator_home_position, actuator_state.size(), home_duration, positions_t, velocities_t, accelerations_t, duration_t);

    // Move the joints of the actuator depending on the interpolation means selected                                                  // Linear interpolation
   move_to_position(actuator_client, actuator_joint_names, home_duration, open_hand, hand, hand_topic, "home", actuator_home_position);

    return 1;


}

/*********************************************************************************

setOdometryPose

Initialize the pose returned by the callback that services the subscription to the odom topic         
                                                                                                       
Odometry provides relative position orientation. Since we can't assume what odometry data is published
on the odom topic on start up, we use two extra sets of variables for the x, y, and theta values:     
adjustment variables and current variables.                                                           
                                                                                                      
We set the values of the adjustment variables to be the difference between                            

(a) the values associated with the start pose, and                                                    
(b) the values published on the odom topic on start up (or whenever we reinitialize the odometry),    
                                                                                                      
The callback then sets the values of the current variables as follows.

- the current x and y values are set to the sum of the adjustment x and y values and the odom x and y values 
  (this effectively translates the odom x and y values by the adjustment x and y values) 

- these translated values are then rotated about the Z axis by an angle equal to the difference 
  between the start theta value  and the odom theta value

- the current theta value is set to be the sum of the adjustment theta value and the odom theta value                                                            

**********************************************************************************/

void setOdometryPose(double x, double y, double theta)
{

   bool debug = false;

   sleep(1); // allow time for messages to be published on the odom topic
   ros::spinOnce();

   /* store the initial pose */
   initial_x = x;
   initial_y = y;
   initial_theta = theta;

   /* calculate the adjustment to the pose, i.e. the difference between the initial pose and odometry pose */
   adjustment_x = x - odom_x;
   adjustment_y = y - odom_y;
   adjustment_theta = theta - odom_theta;

   // sleep(1); // allow time for adjusted  messages to be published on the odom topic
   // ros::spinOnce();

   if (debug)
   {
      printf("setOdometryPose: odom_x,y,theta %.2f %.2f %.2f  adjustment_x, y, theta %.2f %.2f %.2f\n", odom_x, odom_y, odom_theta,
             adjustment_x, adjustment_y, adjustment_theta);
   }
}


int move_robot(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta, ros::Publisher velocity_publisher, ros::Rate rate, bool debug)
{

   double                x_waypoint;
   double                y_waypoint;
   double                theta_waypoint;
    /* Path planning is done, let's navigate */
   /* ====================================  */

   // If there is no physical robot, insert code to publish to simulator robot.
   // For now, it just exits the program

   if (locomotionParameterData.robot_available == false)
   {
      printf("No physical robot present\n");
      return 0;
   }

   /* initialize the odometry */

//    setOdometryPose(start_x, start_y, start_theta);
   // ros::spin();
   
   /* 
      Compute the waypoints in robot frame of reference and navigate using MIMO
      Set waypointFlag to true so as to ensure that robot doesn't stop at waypoints and not worry about goal orientation yet
    */
   waypointFlag = true;

   convert_pixel_to_world(valid_waypoints[0].x, valid_waypoints[0].y, x_waypoint, y_waypoint, room_width, room_height, image_width, image_height);
   theta_waypoint = valid_waypoints[0].theta;
   if(isnan(theta_waypoint)){
      theta_waypoint = start_theta;
   }
//    printf("Valid waypoint = %.3f, %.3f, %.3f\n", x_waypoint, y_waypoint, theta_waypoint);
//    goToPoseDQ(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate);
   goToPoseMIMO(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate, waypointFlag);

   for(int k = 1; k < valid_waypoints.size(); k++)
   {
      convert_pixel_to_world(valid_waypoints[k].x, valid_waypoints[k].y, x_waypoint, y_waypoint, room_width, room_height, image_width, image_height);
      theta_waypoint = valid_waypoints[k].theta;
      // if (debug)
        //  printf("Valid waypoint = %.3f, %.3f, %.3f\n", x_waypoint, y_waypoint, theta_waypoint);

      // goToPoseDQ(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate);
      goToPoseMIMO(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate, waypointFlag);
   }
   // Robot arrived at destination (or one waypoint before destination)
   // Then navigate to the final goal pose
   // Set waypointFlag to false to enable robot stop at final destination and correct final orientation to the goal orientation
   waypointFlag = false;
        // printf("Valid waypoint = %.3f, %.3f, %.3f\n", goal_x, goal_y, goal_theta);
   // // goToPoseMIMO(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate, waypointFlag);
   goToPoseDQ(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate);

   return 1;

}


void save_waypoint_map(vector<int> compressionParams, Mat mapImageLarge, string fileName){
    char path_and_input_filename[MAX_FILENAME_LENGTH]        = "";
    std::string navigation_pathway_filename = fileName;  // name of the output pathway image

    /* Combine the name for the output pathway image*/
    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/robotNavigation/data/"); 
    strcat(path_and_input_filename, navigation_pathway_filename.c_str());

    cv::imwrite(path_and_input_filename, mapImageLarge, compressionParams);   // write the image to a file
    printf("Navigation pathway image is saved in %s\n", path_and_input_filename);
    //imshow(mapWindowName, mapImageLarge); // Display map image
}

/******************************************************************************

goToPoseDQ

Use the divide and conquer algorithm to drive the robot to a given pose

*******************************************************************************/

void goToPoseDQ(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate)
{

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;
   double current_linear_velocity = 0;

   int number_of_ramp_up_steps = 20;

   int mode; // GOING or ORIENTING

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   mode = ORIENTING; // divide and conquer always starts by adjusing the heading

   position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                            (goal_y - current_y) * (goal_y - current_y));

   while ((position_error > locomotionParameterData.position_tolerance_goal) && ros::ok())
   {

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback and publish the pose on the odom topic

      position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                            (goal_y - current_y) * (goal_y - current_y));

      goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
      // printf("Current theta %.3f\n", current_theta);
      angle_error = goal_direction - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,             */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radians or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                  */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      // if (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) {
      if (((mode == ORIENTING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)) || // low angular tolerance when orienting to get the best initial heading
          ((mode == GOING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_going)))
      { // high angular tolerance when going so we don't have to correct the heading too often

         /* if the robot is not oriented correctly, adjust the heading */

         if (debug)
            printf("Orienting\n");

         mode = ORIENTING; // reset mode from GOING to ORIENTING to ensure we use the lower angular tolerance when reorienting

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         msg.linear.x = 0;

         angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
            msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
            msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
            msg.angular.z = angular_velocity;
      }
      else if (position_error > locomotionParameterData.position_tolerance)
      {

         mode = GOING;

         /* if the robot has not reached the goal, adjust the distance */

         if (debug)
            printf("Going\n");

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         linear_velocity = locomotionParameterData.position_gain_dq * position_error;

         if (linear_velocity < locomotionParameterData.min_linear_velocity)
            linear_velocity = locomotionParameterData.min_linear_velocity;
         else if (linear_velocity > locomotionParameterData.max_linear_velocity)
            linear_velocity = locomotionParameterData.max_linear_velocity;

         /* if stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

         if (current_linear_velocity == 0)
         {

            for (int i = 1; i < number_of_ramp_up_steps; i++)
            {
                msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
                msg.angular.z = 0;

                if (debug)
                {
         printf("Ramping up velocity\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n", msg.linear.x, msg.angular.z);
                }

                pub.publish(msg); // Publish the message

                rate.sleep(); // Wait until it's time for another iteration
            }
            current_linear_velocity = linear_velocity;
         }

         msg.linear.x = linear_velocity;
         msg.angular.z = 0;
      }

      if (debug)
      {
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

   }

   /* the robot has reached the destination so             */
   /* adjust the orientation to match the goal orientation */
    angle_error = goal_theta - current_theta;

   while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok())
   {

      if (debug)
         printf("Orienting\n");

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback

      angle_error = goal_theta - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      msg.linear.x = 0;

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */

      angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
         msg.angular.z = angular_velocity;

      if (debug)
      {
         printf("Orienting\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      if((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)){

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration
      }

   }

   msg.linear.x = 0;
   msg.angular.z = 0;

   pub.publish(msg); // Publish the message
   // rate.sleep(); // Wait until it's time for another iteration


}


void stabilize_waist()
{
    naoqi_bridge_msgs::JointAnglesWithSpeed msg;
    msg.joint_names.push_back("HipRoll");
    msg.joint_angles.push_back(0.1);  // Set hip pitch to 0 (upright position)
    msg.joint_names.push_back("HipPitch");
    msg.joint_angles.push_back(-0.1);  // Set hip roll to 0 (upright position)
    msg.joint_names.push_back("KneePitch");
    msg.joint_angles.push_back(0.0);  // Set knee pitch to 0 (upright position)
    msg.speed = 0.2;  // Adjust speed as needed
    msg.relative = 0;  // Absolute position

    navigation_pelvis_publisher.publish(msg);

      // trajectory_msgs::JointTrajectory trajectory_msg;
      // trajectory_msg.joint_names.push_back("HipPitch");
      // trajectory_msg.joint_names.push_back("HipRoll");
      // trajectory_msg.joint_names.push_back("KneePitch");

      // trajectory_msgs::JointTrajectoryPoint trajectory_msg_point;
      // trajectory_msg_point.positions.push_back(-0.5);
      // trajectory_msg_point.positions.push_back(0.0);
      // trajectory_msg_point.positions.push_back(0.0);

      // trajectory_msg_point.time_from_start = ros::Duration(0.5);

      // trajectory_msg.points.push_back(trajectory_msg_point);

      // navigation_pelvis_publisher.publish(trajectory_msg);
}

void stabilize_waist_continuously()
{
    ros::Rate rate(10);  // Adjust the rate as needed
    while (ros::ok())
    {
      //   if (is_moving)
        {
            stabilize_waist();
        }
        rate.sleep();
    }
}




void goToPoseMIMO(double x, double y, double theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
 {

   bool debug = false;

   geometry_msgs::Twist msg;

   double start_x;
   double start_y;
   double start_theta;

   double goal_x;
   double goal_y;
   double goal_theta;

   double goal_direction;

   double position_error;
   double angle_error;

   double angular_velocity;
   double linear_velocity;

   static double current_linear_velocity = 0; // make this static so that it's valid on the next call

   int number_of_ramp_up_steps = 20;

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

   while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok())
   {

     /* get the current pose */

     ros::spinOnce(); // Let ROS take over to handle the callback

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
     /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
     /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     /* set linear and angular velocities, taking care not to use values that exceed maximum values */
     /* or use values that are less than minimum values needed to produce a response in the robot   */

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     /* don't slow down if driving to a waypoint */

     if (waypoints)
     {
         linear_velocity = locomotionParameterData.max_linear_velocity / 2;
     }
     else
     {
         linear_velocity = locomotionParameterData.position_gain_mimo * position_error;
     }

     if (linear_velocity < locomotionParameterData.min_linear_velocity)
         msg.linear.x = locomotionParameterData.min_linear_velocity;
     else if (linear_velocity > locomotionParameterData.max_linear_velocity)
         msg.linear.x = locomotionParameterData.max_linear_velocity;

     /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

     if (current_linear_velocity == 0)
     {

         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
      msg.linear.x = (double)linear_velocity * ((double)i / (double)number_of_ramp_up_steps);
      msg.angular.z = 0;

      if (debug)
      {
              printf("Ramping up velocity\n");
              // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
              // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
              printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
              printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;

     if (debug)
     {
         printf("Going\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }

     pub.publish(msg); // Publish the message

     rate.sleep(); // Wait until it's time for another iteration

   }

   /* for the final destination, adjust the orientation to match the goal pose */
   /* ------------------------------------------------------------------------ */

   if (!waypoints)
   {
    angle_error = goal_theta - current_theta;

      while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok())
     {

         /* if the robot has reached the destination             */
         /* adjust the orientation to match the goal orientation */

         /* get the current pose */

         ros::spinOnce(); // Let ROS take over to handle the callback

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         angle_error = goal_theta - current_theta;

         current_linear_velocity = 0;
         msg.linear.x = current_linear_velocity;

         angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
      msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
      msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
      msg.angular.z = angular_velocity;

         if (debug)
         {
      printf("Orienting\n");
      // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
      // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
      printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
      printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
         }

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration

     }

      current_linear_velocity = 0;
      msg.linear.x = current_linear_velocity;

      angular_velocity = 0;
      msg.angular.z = angular_velocity;

      pub.publish(msg); // Publish the message

      // rate.sleep(); // Wait until it's time for another iteration

   }
 }
/*
 *   Function to round a doubleing point number to a specified number of decimal places
 *
 *  @param:
 *     value: the value to be rounded
 *     decimal_places: the number of decimal places
 *  @return:
 *     the rounded value
 * 
 */
double round_floating_point(double value, int decimal_places){
   double rounded_value = 0.0;

   // rounded_value = std::round(value * std::pow(10, decimal_places)) / std::pow(10, decimal_places);
   double multiplier = std::pow(10.0, decimal_places);
   rounded_value = std::roundf(value * multiplier) / multiplier;

   // rounded_value = static_cast<double>(rounded_value);

   return rounded_value;

}
 /* 
 *   Function to convert radians to degrees
 *   This function converts the angle in radians to degrees
 *
 * @param:
 *   radians: the angle in radians
 *
 * @return:
 *   the angle in degrees
 */
double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to double
    return degrees;
}

/* 
 *   Function to convert degrees to radians
 *   This function converts the angle in degrees to radians
 *
 * @param:
 *   degrees: the angle in degrees
 *
 * @return:
 *   the angle in radians
 */
double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to double
    return radians;
}


/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void prompt_and_continue(){
    printf("Press X to quit or Press any key to continue...\n");
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}

void move_robot_actuators_to_default(){
    double gesture_duration = 1.0; // Set the duration of the gesture to 2 seconds
    // Create a control client for the head
    std::string head_topic;
    head_topic = "/pepper_dcm/Head_controller/follow_joint_trajectory";
    ControlClientPtr head_client = create_client(head_topic);

    if(head_client == nullptr){
        ROS_ERROR("Error creating action client for head controller");
        return;
    }
    std::vector<std::string> head_joint_names = {"HeadPitch", "HeadYaw"};// Set the joint names for the head to the specified joint names
    int number_of_joints = head_joint_names.size(); 
    
    // positions for each joint
   //  std::vector<double> head_position = {0.4325826168060303, -0.013805866241455078};
   std::vector<double> head_position = {0.0, 0.0};

    // Create a control client for the left arm
      std::string left_arm_topic;
      left_arm_topic = "/pepper_dcm/LeftArm_controller/follow_joint_trajectory";
      ControlClientPtr left_arm_client = create_client(left_arm_topic);

      if(left_arm_client == nullptr){
          ROS_ERROR("Error creating action client for left arm controller");
          return;
      }

      std::vector<std::string> left_arm_joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"};// Set the joint names for the left arm to the specified joint names

      // positions for each joint
      std::vector<double> left_arm_position = {1.5523885488510132, 0.13959217071533203, -1.2087767124176025, -0.4540584087371826, -0.15497589111328125};
      
      // Create a control client for the right arm
      std::string right_arm_topic;
      right_arm_topic = "/pepper_dcm/RightArm_controller/follow_joint_trajectory";
      ControlClientPtr right_arm_client = create_client(right_arm_topic);

      if(right_arm_client == nullptr){
          ROS_ERROR("Error creating action client for right arm controller");
          return;
      }

      std::vector<std::string> right_arm_joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"};// Set the joint names for the right arm to the specified joint names

      // positions for each joint
      std::vector<double> right_arm_position = {1.5447185039520264, -0.1441941261291504, 1.2133787870407104,  0.44945645332336426, 0.1487560272216797};
      
      // Create a control client for the leg
      std::string leg_topic;
      leg_topic = "//pepper_dcm/Pelvis_controller/follow_joint_trajectory";
      ControlClientPtr leg_client = create_client(leg_topic);

      if(leg_client == nullptr){
          ROS_ERROR("Error creating action client for leg controller");
          return;
      }

      std::vector<std::string> leg_joint_names = {"HipPitch", "HipRoll", "KneePitch"};// Set the joint names for the leg to the specified joint names

      // positions for each joint
      // std::vector<double> leg_position = {0.007669925689697266, -0.018407821655273438, 0.0490872859954834};
      std::vector<double> leg_position = {0.0, 0.0, 0.0};
    
    // Move the head to the specified position
    move_to_position(head_client, head_joint_names, head_position, 
                     left_arm_client, left_arm_joint_names, left_arm_position, 
                     right_arm_client, right_arm_joint_names, right_arm_position, 
                     leg_client, leg_joint_names, leg_position, 
                     gesture_duration);

    // Move the head to the specified position
      // move_one_actuator_to_position(head_client, head_joint_names, gesture_duration, head_position);

}

void move_to_position(ControlClientPtr& head_client, const std::vector<std::string>& head_joint_names, std::vector<double> head_positions, 
                        ControlClientPtr& left_arm_client, const std::vector<std::string>& left_arm_joint_names, std::vector<double> left_arm_positions, 
                        ControlClientPtr& right_arm_client, const std::vector<std::string>& right_arm_joint_names, std::vector<double> right_arm_positions, 
                        ControlClientPtr& leg_client, const std::vector<std::string>& leg_joint_names, std::vector<double> leg_positions, 
                        double duration){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal head_goal;
    trajectory_msgs::JointTrajectory& head_trajectory = head_goal.trajectory;
    head_trajectory.joint_names = head_joint_names;                               // Set the joint names for the actuator to the specified joint names
    head_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    head_trajectory.points[0].positions = head_positions;                         // Set the positions in the trajectory to the specified positions
    head_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal left_arm_goal;
    trajectory_msgs::JointTrajectory& left_arm_trajectory = left_arm_goal.trajectory;
    left_arm_trajectory.joint_names = left_arm_joint_names;                               // Set the joint names for the actuator to the specified joint names
    left_arm_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    left_arm_trajectory.points[0].positions = left_arm_positions;                         // Set the positions in the trajectory to the specified positions
    left_arm_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal right_arm_goal;
    trajectory_msgs::JointTrajectory& right_arm_trajectory = right_arm_goal.trajectory;
    right_arm_trajectory.joint_names = right_arm_joint_names;                               // Set the joint names for the actuator to the specified joint names
    right_arm_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    right_arm_trajectory.points[0].positions = right_arm_positions;                         // Set the positions in the trajectory to the specified positions
    right_arm_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal leg_goal;
    trajectory_msgs::JointTrajectory& leg_trajectory = leg_goal.trajectory;
    leg_trajectory.joint_names = leg_joint_names;                               // Set the joint names for the actuator to the specified joint names
    leg_trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    leg_trajectory.points[0].positions = leg_positions;                         // Set the positions in the trajectory to the specified positions
    leg_trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Send the goal to move the actuator to the specified position
    // head_client->sendGoal(head_goal);
   left_arm_client->sendGoal(left_arm_goal);
   right_arm_client->sendGoal(right_arm_goal);
      leg_client->sendGoal(leg_goal);

    // head_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   left_arm_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   right_arm_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
      leg_client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
   // ros::Duration(duration).sleep();
}

void move_one_actuator_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration));                     // Wait for the actuator to reach the specified position
}
