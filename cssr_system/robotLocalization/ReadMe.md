<div align="center">
  <h1>Robot Localization</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotLocalization` ROS node provides accurate pose estimation capabilities for the Pepper humanoid robot using visual landmark detection and sensor fusion. This node combines ArUco marker detection from RGB and depth cameras with odometry data to deliver robust pose estimation in indoor environments.

The package implements both triangulation (RGB-only) and trilateration (RGB-D) algorithms for absolute pose computation from detected landmarks, then maintains continuous positioning through odometry integration. The system supports periodic automatic pose correction and on-demand pose reset services for reliable localization in dynamic environments.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of this node and its software. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Node
## Physical Robot 
### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

   **Install Intel RealSense SDK and ROS Wrapper (For the Intel realsense camera):**
   
   - Add Intel server to the list of repositories:
      ```bash
      sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
      sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
      ```
   
   - Install the Intel RealSense SDK 2.0 libraries and utilities:
      ```bash
      sudo apt update
      sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-udev-rules
      ```
   
   - Install the ROS wrapper for RealSense cameras:
      ```bash
      # For ROS Noetic:
      sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
      ```
   
   - Verify the installation by connecting your RealSense camera and running:
      ```bash
      realsense-viewer
      ```  

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
         cd .. && source devel/setup.bash && catkin_make
       ```

3. **Install ArUco Library Package (ROS Noetic)**
     ```bash
        sudo apt update
        sudo apt install ros-noetic-aruco ros-noetic-aruco-msgs ros-noetic-aruco-ros
      ``` 

4. **Update Data and Configuration Files:**
   
   Navigate to the configuration files and update them according to your use case and environment setup:

   **Launch File Configuration** (`launch/robotLocalizationLaunchRobot.launch`):
   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `verbose` | Diagnostic info printing | `true`, `false` | `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` | `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` | `true` |
   | `head_yaw_joint_name` | Name of head yaw joint | String | `HeadYaw` |
   | `reset_interval` | Automatic pose reset interval (seconds) | Float | `10.0` |
   | `absolute_pose_timeout` | Timeout for pose validity (seconds) | Float | `300.0` |
   | `camera_info_timeout` | Timeout for retrieving camera intrinsic from camera (seconds) | Float | `15.0` |

   **Landmark (ArUco Markers) Data** (`data/landmarks.json`): These are the positions of ArUco markers placed in the known environment.
   ```json
   "landmarks": [
    {
      "id": 1,
      "x": 5.0,   # X coordinate in meters
      "y": 4.8,   # Y coordinate in meters
      "z": 0.71   # Z coordinate in meters (ideal is camera height)
    },
    {
      "id": 2,
      "x": 2.0,
      "y": 5.4,
      "z": 0.71
    }]
   ```

   **Camera Calibration** (`config/camera_info.json`):
   Default calibration if camera intrinsic is not retrieved automatically from camera.
   ```json
   "camera_info": {
      "fx": 911.6033325195312,   # Focal length X
      "fy": 910.8851318359375,   # Focal length Y
      "cx": 655.0755615234375,   # Principal point X
      "cy": 363.9165954589844    # Principal point Y
   }
   ```
    To retrieve and update these values:
      - Launch the realsense camera
        ```bash
            roslaunch realsense2_camera rs_camera.launch
          ```
      - Echo the camera info topic and retrieve values
        ```bash
            rostopic echo /camera/color/camera_info
          ```

5. **Set up ArUco Markers:**
   
   Place ArUco markers (DICT_4X4_100) in your physical environment at the coordinates specified in `landmarks.json`. Ensure markers are:
   - Clearly visible from robot operating areas
   - At appropriate heights (recommended: camera height ±0.5m)
   - Well-distributed to avoid collinear configurations and acute angles between the markers
   - Properly lit and unobstructed

6. **Run the `robotLocalization` from `cssr_system` package:**
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the `robotLocalization` node.
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system robotLocalizationLaunchRobot.launch
        ```
        
        **Or run the node standalone** (if camera is already launched and maintaining default configurations):
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
        ```
    
      N.B: Running the `robotLocalization` node requires camera topics (`/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`) and odometry topic (`/naoqi_driver/odom`) to be available from the robot or simulation environment.


## Executing an Action
Upon launching the node, the hosted services (`/robotLocalization/set_pose`, `/robotLocalization/reset_pose`) are available and ready to be invoked. This can be verified by running the following command in a new terminal:

```sh
rosservice list | grep /robotLocalization
```

### Service Commands

**Set the Initial Robot Pose**
Use the set pose service to initialize or correct the robot's pose:
```sh
rosservice call /robotLocalization/set_pose x y theta
```

**Reset Pose Using Landmarks**
Trigger absolute pose computation from detected markers:
```sh
rosservice call /robotLocalization/reset_pose
```

**View Detected Markers**
Monitor the annotated marker image to verify detection:
```sh
rosrun image_view image_view image:=/robotLocalization/marker_image
```

**Echo the Pose Topic**
View the pose data published by the localization node:
```sh
rostopic echo /robotLocalization/pose
```

### Service Request Parameters
#### 1. Set Pose Service (`/robotLocalization/set_pose`)
- `x`: X-coordinate in meters (float)
- `y`: Y-coordinate in meters (float)
- `theta`: Orientation in degrees (float)

#### 2. Reset Pose Service (`/robotLocalization/reset_pose`)
- No parameters (triggers automatic pose computation from detected landmarks)

### Sample Invocations
- Set robot at position (2.0, 6.6) facing 0 degrees:
  ```sh
  rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
  ```
- Set robot facing -45 degrees (360 degrees - 45 degrees):
  ```sh
  rosservice call /robotLocalization/set_pose 2.0 6.6 315.0
  ```
- Trigger pose reset from landmarks:
  ```sh
  rosservice call /robotLocalization/reset_pose
  ```

## Monitoring and Debugging

**Enable Verbose Output:**
```sh
rosparam set /robotLocalization/verbose true
```
or in the launch configuration file

**Check Node Status:**
```sh
rosnode info /robotLocalization
```

**Monitor Topic Rates:**
```sh
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
   - Verify landmark placement accuracy (avoid collinear markers and acute angles)

3. **High pose drift:**
   - Reduce reset_interval parameter
   - Check odometry quality
   - Ensure sufficient landmark coverage

<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the robotLocalization node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ioj@andrew.cmu.edu">ioj@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-06-23