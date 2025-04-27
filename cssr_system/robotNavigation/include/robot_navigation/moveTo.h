/*******************************************************************************************************************
*
*  Pepper Navigation: Path Planning and robot navigation for Pepper robot in the Robotics Lab at CMU-Africa
*
*   This is the interface file.
*   For documentation, please see the application file
*
*   Adedayo Akinade
*   26 June 2023
*
*   Audit Trail
*   -----------
*
*
*
*******************************************************************************************************************/


#define ROS
 
#ifndef ROS
   #include <conio.h>
#else
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <sys/select.h>
   #include <termios.h>
   //#include <stropts.h>
   #include <sys/ioctl.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h> // for turtle1/teleport_absolute service
#include <turtlesim/SetPen.h>           // for turtle1/set_pen service
#include <std_srvs/Empty.h>             // for reset and clear services
#include <geometry_msgs/Twist.h>        // For geometry_msgs::Twist
#include <nav_msgs/Odometry.h>          // For nav_msgs::Odometry
#include <iomanip>                      // for std::setprecision and std::fixed

//#include <cv2.h>
//#include <highgui.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/***************************************************************************************************************************

   ROS package name

****************************************************************************************************************************/
#define ROS_PACKAGE_NAME    "pepper_navigation"


/***************************************************************************************************************************

   General purpose definitions 

****************************************************************************************************************************/
#define PI       3.14159
#define TRUE     1
#define FALSE    0
#define BFS      0
#define DIJKSTRA 1
#define ASTAR    2

/***************************************************************************************************************************

   Definitions for reading locomotion parameter data 

****************************************************************************************************************************/

#define MAX_FILENAME_LENGTH 200
#define STRING_LENGTH       200
#define KEY_LENGTH           40
#define NUMBER_OF_KEYS       14
#define GOING                 0  // used to switch between angle_tolerance_going and angle_tolerance_orienting
#define ORIENTING             1

typedef char keyword[KEY_LENGTH];

typedef struct {
   float position_tolerance;
   float angle_tolerance_orienting;
   float angle_tolerance_going;
   float position_gain_dq; 
   float angle_gain_dq; 
   float position_gain_mimo; 
   float angle_gain_mimo;
   float min_linear_velocity;                           // m/s       ... from calibration; less than this and the motors are not actuated
   float max_linear_velocity;                           // m/s       ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
   float min_angular_velocity;                          // radians/s ... from calibration; less than this and the motors are not actuated
   float max_angular_velocity;                          // radians/s ... see "Commanding your Create" on  https://github.com/AutonomyLab/create_robot
   float clearance;                                     // m         ... required clearance between the robot and an obstacle
   int   shortest_path_algorithm;                       // BFS, Dijkstra, ASTAR
   bool  robot_available;                               // true/false... determine if a physical robot is available
} locomotionParameterDataType;

/***************************************************************************************************************************

   Definitions for paths and waypoints 

****************************************************************************************************************************/

#define MAX_NUMBER_OF_WAYPOINTS   100
#define MAX_NUMBER_OF_PATHPOINTS 5000

/* of course we should be using linked lists for these data structures */

typedef struct  {
   int x;
   int y;
} pointType;

typedef struct {
        pointType point[MAX_NUMBER_OF_PATHPOINTS];  
        int       numberOfPoints;               
} pathType;


typedef struct  {
   int x;
   int y;
   float theta;
} waypointType;

typedef struct {
        waypointType point[MAX_NUMBER_OF_WAYPOINTS];  
        int          numberOfPoints;               
} waypointArrayType;


/***************************************************************************************************************************

   Definitions for the queue data structure 

****************************************************************************************************************************/
#define QUEUESIZE  10000                /* this limits the number of vertices waiting to be processed                      */
                                        /* it is determined by the size of the frontier of the traversal wavefront         */
					/* this will be on the order of the size of the maximum dimension of the  map      */

/* of course we should be using linked lists for these data structures */

typedef int item_type;

typedef struct {
        item_type q[QUEUESIZE+1];       /* body of queue */
        int first;                      /* position of first element */
        int last;                       /* position of last element */
        int count;                      /* number of queue elements */
} dvqueue;


/***************************************************************************************************************************

   Definitions for the graph data structure

****************************************************************************************************************************/

/* Adjacency list representation of a graph of degree MAXV          */
/*                                                                  */
/* Directed edge (x, y) is represented by edgenode y in x's         */
/* adjacency list. Vertices are numbered 1 .. MAXV                  */

#define MAX_N      1000           /* maximum dimensions of the map   */
#define MAX_M      1000           /* 5 x 5 metres at 1 cm per cell   */
#define MAXV       (MAX_N * MAX_M)    /* maximum number of vertices */
// #define MAXV       900000    /* maximum number of vertices */

typedef struct edgenode *EDGENODE_PTR;

typedef struct edgenode {
        long int y;              /* adjacent vertex number          */
        bool hidden;             /* hidden / invisible edge         */     
        float weight;            /* edge weight, if any             */
        EDGENODE_PTR next;       /* next edge in list               */
} edgenode;

typedef struct {
        edgenode *edges[MAXV+1]; /* adjacency info: list of edges   */
      //   unsigned int degree[MAXV+1];      /* number of edges for each vertex */
        long int nvertices;      /* number of vertices in graph     */
        long int nedges;         /* number of edges in graph        */
        bool directed;           /* is the graph directed?          */
} graph;


/***************************************************************************************************************************

   Function declarations for the graph data structure 

****************************************************************************************************************************/

void initialize_graph(graph *g, bool directed);
bool read_graph(FILE *fp_in, graph *g, bool directed);
bool map_to_graph(graph *g, bool directed, Mat map, int algorithm);
bool find_path(graph *g, int start, int end, int parents[],  Mat map, int algorithm, pathType &path);
bool find_path(graph *g, int start, int end,                 Mat map, int algorithm, pathType &path);
void compute_waypoints(pathType path, waypointArrayType &waypoints);
void insert_edge(graph *g, int x, int y, bool directed, float w);
void print_graph(graph *g);
void initialize_search(graph *g);
void bfs(graph *g, int start);
void dijkstra(graph *g, int start);
float calculate_heuristic(int start, int goal, Mat map);
void astar(graph *g, int start, int goal, Mat map) ;
void process_vertex_late(int v);
void process_vertex_early(int v);
void process_edge(int x, int y);
bool find_path(int start, int end, int parents[]);
bool print_path(FILE *fp_out, graph *g, int start, int end);
bool print_path(FILE *fp_out, int start, int end, int parents[]);
bool find_path(graph *g, int start, int end);

/***************************************************************************************************************************

   Function declarations for the queue data structure 

****************************************************************************************************************************/

void init_queue(dvqueue *q);
void enqueue(dvqueue *q, item_type x);
item_type dequeue(dvqueue *q);
item_type headq(dvqueue *q);
int empty_queue(dvqueue *q);
void print_queue(dvqueue *q);


/***************************************************************************************************************************

   Function declarations for mapping between the graph and map data structures

****************************************************************************************************************************/

int row_number(int vertex_number, int number_of_columns);
int column_number(int vertex_number, int number_of_columns);
int vertex_number(int row, int column, int number_of_columns);


/***************************************************************************************************************************

   Function declarations for mobile robot control 

****************************************************************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry& msg); // callback: executed each time a new message arrives on the odom topic
void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData);
void readObstacleData(char filename[], Mat &map);
void findMinimumVelocities(ros::Publisher pub, ros::Rate rate, float max_linear_velocity,  float max_angular_velocity);
void setOdometryPose(float x, float y, float z);
void goToPoseDQ     (float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate);
void goToPoseMIMO   (float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints);


/***************************************************************************************************************************

   General purpose function declarations 

****************************************************************************************************************************/

void display_error_and_exit(char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, char message[]);
int  signnum(float x); // return the sign of a number

#ifdef ROS
   int _kbhit();
#endif
