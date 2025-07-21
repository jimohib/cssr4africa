<div align="center">
  <h1>Robot Localization</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `robotLocalization` ROS node provides accurate pose estimation capabilities for the Pepper humanoid robot using visual landmark detection and sensor fusion. This node combines ArUco marker detection from RGB and depth cameras with odometry data to deliver robust 6-DOF pose estimation in indoor environments.

The package implements both triangulation (RGB-only) and trilateration (RGB-D) algorithms for absolute pose computation from detected landmarks, then maintains continuous positioning through odometry integration. The system supports periodic automatic pose correction and on-demand pose reset services for reliable localization in dynamic environments.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Node
## Physical Robot 
### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

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
       
3. **Update Configuration Files:**
   
   Navigate to the configuration files and update them according to your environment setup:

   **Launch File Configuration** (`launch/robotLocalization.launch`):
   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `verbose` | Diagnostic info printing | `true`, `false` | `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` | `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` | `true` |
   | `head_yaw_joint_name` | Name of head yaw joint | String | `HeadYaw` |
   | `reset_interval` | Automatic pose reset interval (seconds) | Float | `10.0` |
   | `absolute_pose_timeout` | Timeout for pose validity (seconds) | Float | `300.0` |

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
   ```

   **Camera Calibration** (`config/camera_info.yaml`):
   ```yaml
   camera_info:
     fx: 525.0  # Focal length X
     fy: 525.0  # Focal length Y
     cx: 319.5  # Principal point X
     cy: 239.5  # Principal point Y
   ```

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>

4. **Set up ArUco Markers:**
   
   Place ArUco markers (DICT_4X4_100) in your environment at the coordinates specified in `landmarks.yaml`. Ensure markers are:
   - Clearly visible from robot operating areas
   - At appropriate heights (recommended: camera height ±0.5m)
   - Well-distributed to avoid collinear configurations
   - Properly lit and unobstructed

5. **Run the `robotLocalization` from `cssr_system` package:**
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch cssr_system cssrSystemLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the `robotLocalization` node.
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system robotLocalization.launch
        ```
        
        **Or run the node standalone** (if camera is already launched):
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
        ```
    
      N.B: Running the `robotLocalization` node requires camera topics (`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`) and odometry topic (`/pepper_dcm/odom`) to be available from the robot or simulation environment.

## Simulator Robot

### Steps
1. **Install the required software components:**
   
   Set up the development environment for controlling the Pepper robot in the simulated environment. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

2. **Clone and build the project (if not already cloned):**
   - Move to the source directory of the workspace:
      ```bash
      cd $HOME/workspace/pepper_sim_ws/src
      ```
   - Clone the `CSSR4Africa` software from the GitHub repository:
      ```bash
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files:
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```

3. **Update Configuration Files:**
   
   Navigate to the configuration files located at `~/workspace/pepper_sim_ws/src/cssr4africa/robotLocalization/config/` and update the configuration according to your simulation environment setup.

   **Launch File Configuration** (`launch/robotLocalization.launch`):
   | Parameter | Description | Values | Default |
   |-----------|-------------|---------|---------|
   | `verbose` | Diagnostic info printing | `true`, `false` | `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` | `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` | `true` |
   | `reset_interval` | Automatic pose reset interval (seconds) | Float | `10.0` |

   - To execute the localization on the simulator platform, ensure the landmark configuration matches your simulation environment.

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. Otherwise, the recommended values are the ones already set in the configuration file.</span>
  </div>

4. **Run the `robotLocalization` from `cssr_system` package:**:
   
   Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the simulator robot:
        ```bash
          roslaunch cssr_system cssrSystemLaunchSimulator.launch
        ```
    - Open a new terminal to launch the `robotLocalization` node.
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && roslaunch cssr_system robotLocalization.launch
        ```
        
        **Or run the node standalone** (if camera is already launched):
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && rosrun cssr_system robotLocalization
        ```

## Executing an Action
Upon launching the node, the hosted services (`/robotLocalization/set_pose`, `/robotLocalization/reset_pose`) are available and ready to be invoked. This can be verified by running the following command in a new terminal:

```sh
rosservice list | grep /robotLocalization
```

### Service Commands

**Set the Initial Robot Pose**
Use the set pose service to initialize or correct the robot's pose:
```sh
rosservice call /robotLocalization/set_pose -- x y theta
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
- Set robot facing -45 degrees (use quotes for negative values):
  ```sh
  rosservice call /robotLocalization/set_pose "{x: 2.0, y: 6.6, theta: -45.0}"
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
   - Verify landmark placement accuracy

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