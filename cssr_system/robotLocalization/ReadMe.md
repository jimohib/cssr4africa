<div align="center">
  <h1>Robot Localization</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotLocalization` ROS node provides accurate pose estimation capabilities for the Pepper humanoid robot using visual landmark detection and sensor fusion. This node combines ArUco marker detection from RGB and depth cameras with odometry data to deliver robust 6-DOF pose estimation in indoor environments.

The package implements both triangulation (RGB-only) and trilateration (RGB-D) algorithms for absolute pose computation from detected landmarks, then maintains continuous positioning through odometry integration. The system supports periodic automatic pose correction and on-demand pose reset services for reliable localization in dynamic environments.

To accommodate diverse deployment scenarios, parameters such as depth usage, head movement compensation, landmark configurations, and verbose output are fully configurable. This package is designed for seamless integration with physical Pepper robots and provides essential positioning data for autonomous navigation.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Node

### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf). 

2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
      cd $HOME/workspace/pepper_rob_ws/src
      ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files
      ```bash
      cd ..
      ```
      ```bash
      catkin_make
      ```
      ```bash
      source devel/setup.bash 
      ```
       
3. **Update Configuration Files:**
   
   The launch file (`robotLocalization.launch`) contains the main configuration parameters. You can modify them directly in the launch file or create parameter files:

   **Launch File Configuration** (`launch/robotLocalization.launch`):
   ```xml
   <launch>
       <!-- Launch RealSense camera node -->
       <include file="$(find realsense2_camera)/launch/rs_camera.launch">
           <arg name="enable_color" value="true" />
           <arg name="enable_depth" value="true" />
           <arg name="color_width" value="1280" />
           <arg name="color_height" value="720" />
           <arg name="color_fps" value="30" />
           <arg name="depth_width" value="848" />
           <arg name="depth_height" value="480" />
           <arg name="depth_fps" value="30" />
           <arg name="align_depth" value="true" />
       </include>

       <!-- Static transformations -->
       <node pkg="tf" type="static_transform_publisher" name="odom_to_map" 
             args="0 0 0 0 0 0 map odom 100" />
       <node pkg="tf" type="static_transform_publisher" name="head_to_camera_link"
             args="0.1 0.0 0.2 0.0 0.0 0.0 Head camera_link 100" />

       <!-- Launch robotLocalization node -->
       <node pkg="cssr_system" type="robotLocalization" name="robotLocalization" output="screen">
           <param name="verbose" value="false" />
           <param name="use_depth" value="false" />
           <param name="use_head_yaw" value="true" />
           <param name="head_yaw_joint_name" value="HeadYaw" />
           <param name="reset_interval" value="10.0" />
           <param name="absolute_pose_timeout" value="300.0" />
           <param name="config_file" value="$(find cssr_system)/robotLocalization/config/landmarks.yaml" />
           <param name="topics_file" value="$(find cssr_system)/robotLocalization/data/pepperTopics.dat" />
           <param name="camera_info_file" value="$(find cssr_system)/robotLocalization/config/camera_info.yaml" />
           <param name="camera_info_timeout" value="15.0" />
           <param name="map_frame" value="map" />
           <param name="odom_frame" value="odom" />
       </node>
   </launch>
   ```

   **Main Configuration Parameters:**
   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `verbose` | Diagnostic info printing | `true`, `false` | `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` | `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` | `false` |
   | `head_yaw_joint_name` | Name of head yaw joint | String | `HeadYaw` |
   | `reset_interval` | Automatic pose reset interval (seconds) | Float | `30.0` |
   | `absolute_pose_timeout` | Timeout for pose validity (seconds) | Float | `300.0` |
   | `config_file` | Landmark configuration file | String | `config/landmarks.yaml` |
   | `topics_file` | Robot topic mapping file | String | `data/pepperTopics.dat` |
   | `camera_info_file` | Camera calibration file | String | `config/camera_info.yaml` |
   | `camera_info_timeout` | Camera info wait timeout (seconds) | Float | `10.0` |

   **Landmark Configuration** (`config/landmarks.yaml`):
   ```yaml
   landmarks:
     - id: 0
       x: 1.0    # X coordinate in meters
       y: 2.0    # Y coordinate in meters  
       z: 1.225  # Z coordinate in meters (camera height)
     - id: 1
       x: 3.0
       y: 2.0
       z: 1.225
     # Add more landmarks as needed
   ```

   **Camera Calibration** (`config/camera_info.yaml`):
   ```yaml
   camera_info:
     fx: 525.0  # Focal length X
     fy: 525.0  # Focal length Y
     cx: 319.5  # Principal point X
     cy: 239.5  # Principal point Y
   ```

4. **Set up ArUco Markers:**
   
   Place ArUco markers (DICT_4X4_100) in your environment at the coordinates specified in `landmarks.yaml`. Ensure markers are:
   - Clearly visible from robot operating areas
   - At appropriate heights (recommended: camera height ±0.5m)
   - Well-distributed to avoid collinear configurations
   - Properly lit and unobstructed

5. **Run the `robotLocalization` from the `cssr_system` package:**
   
   Follow these steps, running in different terminals:
    -  Source the workspace in first terminal:
        ```bash
        cd $HOME/workspace/pepper_rob_ws
        ```
        ```bash
        source devel/setup.bash
        ```
    -  ***Launch and connect to the robot*** (make sure the robot is connected to the network and the robot is powered on, and the robot has to be on the same network with the computer being used to connect to the robot):

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
            <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
            <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code>, <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set in the unit test launch file based on your robot's configuration and your computer's network interface. </span>
         </div>

         ```bash
         roslaunch unit_tests robotLocalizationLaunchTestRobot.launch 
        ```
        
    - **Alternative: Launch with integrated camera** (recommended for standalone operation):
        ```bash
        cd $HOME/workspace/pepper_rob_ws 
        ```
        ```bash
        source devel/setup.bash 
        ```
        ```bash
        roslaunch cssr_system robotLocalization.launch
        ```
        
         
    - Open a new terminal to launch the `robotLocalization` node:
        ```bash
        cd $HOME/workspace/pepper_rob_ws 
        ```
        ```bash
        source devel/setup.bash 
        ```
    - To run the `robotLocalization` node with RealSense camera, use the launch file:
        ```bash
        roslaunch cssr_system robotLocalization.launch
        ```
        
        **Or run the node standalone** (if camera is already launched):
        ```bash
        rosrun cssr_system robotLocalization
        ```

         <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
          <span style="color: #cccccc;">The launch file automatically starts the RealSense camera with proper configuration and sets up required TF transforms. Running the <code>robotLocalization</code> node requires camera topics (<code>/camera/color/image_raw</code>, <code>/camera/depth/image_raw</code>, <code>/camera/color/camera_info</code>) and odometry topic (<code>/pepper_dcm/odom</code>) to be available from the robot or simulation environment. </span>
         </div>

6. **Using the `robotLocalization` Services:**
  Upon launching the node, the hosted services are available and ready to be invoked. This can be verified by running the following command in a new terminal:

     ```bash
      rosservice list | grep /robotLocalization
      ```
   **Echo the Pose Topic**
   View the pose data published by the localization node:
   ```bash
   rostopic echo /robotLocalization/pose
   ```
   
   **Set the Initial Robot Pose**
   Use the set pose service to initialize or correct the robot's pose:
   ```bash
   rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
   ```
   
   **Reset Pose Using Landmarks**
   Trigger absolute pose computation from detected markers:
   ```bash
   rosservice call /robotLocalization/reset_pose
   ```
   
   **View Detected Markers**
   Monitor the annotated marker image to verify detection:
   ```bash
   rosrun image_view image_view image:=/robotLocalization/marker_image
   ```

   **Service Parameters:**
   - **Set Pose Service** (`/robotLocalization/set_pose`):
     - `x`: X-coordinate in meters
     - `y`: Y-coordinate in meters  
     - `theta`: Orientation in degrees
   
   - **Reset Pose Service** (`/robotLocalization/reset_pose`):
     - No parameters (triggers automatic pose computation)

   **Sample Service Invocations:**
   ```bash
   # Set robot at position (2.0, 6.6) facing 0 degrees
   rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
   
   # Set robot facing -45 degrees (use quotes for negative values)
   rosservice call /robotLocalization/set_pose "{x: 2.0, y: 6.6, theta: -45.0}"
   
   # Trigger pose reset from landmarks
   rosservice call /robotLocalization/reset_pose
   ```

7. **Launch File Benefits:**
   
   The provided launch file offers several advantages:
   - **Automatic camera setup**: Configures RealSense camera with optimal settings
   - **TF transforms**: Sets up required coordinate frame transformations
   - **Parameter management**: Centralizes all configuration in one file
   - **Single command**: Starts entire localization system with one command
   
   **Customizing the Launch File:**
   ```bash
   # Copy and modify the launch file for your specific setup
   cp $(find cssr_system)/launch/robotLocalization.launch ~/my_localization.launch
   
   # Launch with custom parameters
   roslaunch ~/my_localization.launch
   ```

8. **Monitoring and Debugging:**
   
   **Enable Verbose Output:**
   ```bash
   rosparam set /robotLocalization/verbose true
   ```
   
   **Check Node Status:**
   ```bash
   rosnode info /robotLocalization
   ```
   
   **Monitor Topic Rates:**
   ```bash
   rostopic hz /robotLocalization/pose
   rostopic hz /camera/color/image_raw
   ```

## Troubleshooting

**Common Issues:**

1. **No markers detected:**
   - Check marker visibility and lighting
   - Verify landmark coordinates in config file
   - Ensure markers use DICT_4X4_100 dictionary

2. **Inaccurate pose estimation:**
   - Calibrate camera intrinsics properly
   - Check for marker ID conflicts
   - Verify landmark placement accuracy

3. **High pose drift:**
   - Reduce reset_interval parameter
   - Check odometry quality
   - Ensure sufficient landmark coverage

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ioj@andrew.cmu.edu">ioj@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-06-23