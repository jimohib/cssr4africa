<div align="center">
  <h1>Robot Localization Unit Test</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `robotLocalization` node within the CSSR4Africa project (`cssr_system` package). The unit tests validate the communication, computation, and configuration functionality of the component across three different testing environments: physical robot, simulator, and test harness with controlled data. The results are logged in the file `~/workspace/pepper_rob_ws/src/unit_tests/robotLocalizationTest/data/robotLocalizationTestOutput.dat` for the physical robot and `~/workspace/pepper_sim_ws/src/unit_tests/robotLocalizationTest/data/robotLocalizationTestOutput.dat` for the simulator robot.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Unit Test 
## Physical Robot 
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

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
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/robotLocalizationTest/config/robotLocalizationTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `verbose` | Diagnostic info printing | `true`, `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` |
   | `reset_interval` | Automatic pose reset interval | `5.0`, `10.0`, `30.0` |

   - To execute the tests on the physical platform, change the first line of `robotLocalizationTestConfiguration.ini` file in the config folder to "`platform robot`". 
   - Set up ArUco markers (DICT_4X4_100) in your environment at the coordinates specified in the test landmark configuration.
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. Otherwise, the preferred values are the ones already set in the `robotLocalizationTestConfiguration.ini` file.</span>
  </div>

4. **Run the `robotLocalizationTest` from the`unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests robotLocalizationLaunchTestRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the robotLocalizationTest (which launches the robotLocalization node and run tests on it). This creates drivers for camera topics and stubs for pose validation.
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch unit_tests robotLocalizationLaunchTestHarness.launch
        ```

## Simulator Robot
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_sim_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_sim_ws/src/unit_tests/robotLocalizationTest/config/robotLocalizationTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `platform` | Target platform | `robot` or `simulator` |
   | `verbose` | Diagnostic info printing | `true`, `false` |
   | `use_depth` | Enable depth-based trilateration | `true`, `false` |
   | `use_head_yaw` | Compensate for head rotation | `true`, `false` |
   | `reset_interval` | Automatic pose reset interval | `5.0`, `10.0`, `30.0` |

   - To execute the tests on the simulator platform, change the first line of `robotLocalizationTestConfiguration.ini` file in the config folder to "`platform simulator`". 
   - Ensure the landmark configuration matches your simulation environment.
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. Otherwise, the preferred values are the ones already set in the `robotLocalizationTestConfiguration.ini` file.</span>
  </div>

4. **Run the `robotLocalizationTest` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the simulator robot:
        ```bash
          roslaunch unit_tests robotLocalizationLaunchTestSimulator.launch
        ```

    - Open a new terminal to launch the robotLocalizationTest (which launches the robotLocalization node and run tests on it). This creates drivers for camera topics and stubs for pose validation.
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && roslaunch unit_tests robotLocalizationLaunchTestHarness.launch
        ```

## Tests Executed
### Test A: Communication Functionality Validation
The Test A contains the following:
  - `Test A_1`: Input Data Processing (RGB images, depth images, odometry, camera calibration)
  - `Test A_2`: Output Data Generation (pose publication, marker image publication)

The robot is expected to perform the following:
- Process RGB images at 5-30 Hz and detect ArUco markers
- Handle depth images for trilateration when enabled
- Publish pose estimates at 1-10 Hz
- Generate annotated marker images showing detection results

### Test B: Computation Functionality Validation
The Test B contains the following:
  - `Test B_1`: Triangulation Algorithm Testing (RGB-only pose computation)
  - `Test B_2`: Trilateration Algorithm Testing (RGB-D pose computation with depth)

The robot is expected to perform the following:
- Achieve position accuracy within ±0.2m for triangulation mode
- Achieve position accuracy within ±0.5m for trilateration mode
- Maintain orientation accuracy within ±5° under good conditions
- Perform sensor fusion between odometry and landmark detection

### Test C: Configuration Functionality Validation
The Test C contains the following:
  - `Test C_1`: Parameter Impact Testing (verbose mode, depth usage, reset intervals)
  - `Test C_2`: Landmark and Camera Configuration Testing (marker positions, camera calibration)

The robot is expected to perform the following:
- Respond appropriately to configuration parameter changes
- Handle different landmark configurations and camera calibrations
- Demonstrate robust performance across various parameter settings
- Show degraded performance with intentionally incorrect configurations

## Results
The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/robotLocalizationTest/data/robotLocalizationTestOutput.dat` file for the physical robot and `~/workspace/pepper_sim_ws/src/unit_tests/robotLocalizationTest/data/robotLocalizationTestOutput.dat` file for the simulator robot. It contains the test ran, the input commands and the status of the test. Below is the output of the test when some sample configuration is set:

```
Robot Localization Test Report: robot
====================================
Date: 2025-07-15 14:03:22

Test A: Communication Functionality
	Input Data Processing
		RGB Image Processing     : PASSED
		Depth Image Processing   : PASSED  
		Odometry Processing      : PASSED
		Camera Info Processing   : PASSED
	Output Data Generation
		Pose Publication         : PASSED
		Marker Image Publication : PASSED
	Result: PASSED

Test B: Computation Functionality  
	Triangulation Algorithm
		Position Accuracy        : PASSED (±0.15m)
		Orientation Accuracy     : PASSED (±4.2°)
	Trilateration Algorithm  
		Position Accuracy        : PASSED (±0.35m)
		Orientation Accuracy     : PASSED (±4.8°)
	Sensor Fusion
		Odometry Integration     : PASSED
		Drift Correction        : PASSED
	Result: PASSED

Test C: Configuration Functionality
	Parameter Impact Testing
		Verbose Mode Toggle      : PASSED
		Depth Usage Toggle       : PASSED
		Reset Interval Change    : PASSED
		Head Yaw Compensation    : PASSED
	Configuration Validation
		Landmark Positions       : PASSED
		Camera Calibration       : PASSED
		Invalid Config Handling  : PASSED
	Result: PASSED

Performance Metrics:
	Pose Publication Rate    : 8.5 Hz
	Position Accuracy (RGB)  : ±0.15m
	Position Accuracy (RGB-D): ±0.35m  
	Orientation Accuracy     : ±4.2°
	Landmark Detection Rate  : 94%
	System Latency          : 285ms

Overall Test Result: PASSED
```

## Performance Metrics and Success Criteria

### Expected Performance Metrics
- **Pose Publication Rate**: 1-10 Hz
- **Position Accuracy**: ±0.2m (RGB), ±0.5m (RGB-D)
- **Orientation Accuracy**: ±5° (RGB), ±5° (RGB-D)
- **Landmark Detection Rate**: >90% when 3+ markers visible
- **System Latency**: <500ms from image to pose

### Success Criteria
1. **Startup**: Copyright message displayed, all subscriptions confirmed
2. **Communication**: All input topics received, output topics publishing
3. **Computation**: Pose estimates within accuracy tolerances
4. **Configuration**: Parameter changes produce expected behavior
5. **Robustness**: Graceful handling of missing data, invalid configurations
6. **Heartbeat**: "robotLocalization: running" message every 10 seconds

### Test Commands for Manual Validation
```bash
# Monitor pose output
rostopic echo /robotLocalization/pose

# View marker detection
rosrun image_view image_view image:=/robotLocalization/marker_image

# Test services
rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
rosservice call /robotLocalization/reset_pose

# Check performance
rostopic hz /robotLocalization/pose
rostopic hz /camera/color/image_raw
```

## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the robotLocalizationTest node, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf" style="color: #66b3ff;">D4.2.4 Robot Localization</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ioj@andrew.cmu.edu">ioj@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-07-15