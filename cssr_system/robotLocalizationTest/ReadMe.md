<div align="center">
  <h1>Robot Localization Unit Test</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides comprehensive unit tests for the `robotLocalization` node within the CSSR4Africa project (`cssr_system` package). The unit tests validate the communication, computation, and configuration functionality of the component across two different testing environments: physical robot and test harness with controlled data. 

The test results are logged to provide detailed validation reports and performance metrics for the localization system.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of this node and its software. The deliverable report can be found in [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf).

# Run the Robot Localization Unit Test

## Physical Robot

### Steps

1. **Install the required software components:**

   Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf).

   **Install Intel RealSense SDK and ROS Wrapper:**
   
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
   
   - Verify the installation:
      ```bash
      realsense-viewer
      ```   

2. **Clone and build the project (if not already cloned):**
   ```bash
   # Navigate to workspace
   cd $HOME/workspace/pepper_rob_ws/src
   
   # Clone repository
   git clone https://github.com/cssr4africa/cssr4africa.git
   
   # Build
   cd .. && source devel/setup.bash && catkin_make
   ```

3. **Install ArUco Library Package:**
   ```bash
   sudo apt update
   sudo apt install ros-noetic-aruco ros-noetic-aruco-msgs ros-noetic-aruco-ros
   ```

## Test Configuration

### JSON Configuration File

Navigate to `config/robotLocalizationTestConfiguration.json` and update according to your test requirements:

| Parameter | Description | Values | Default |
|-----------|-------------|---------|---------|
| `verboseMode` | Diagnostic info printing | `true`, `false` | `true` |
| `unitTestMode` | Enable unit test features | `true`, `false` | `true` |
| `camera` | RGB camera source | `"RGBRealSense"`, `"FrontCamera"` | `"RGBRealSense"` |
| `depthCamera` | Depth camera source | `"DepthRealSense"` | `"DepthRealSense"` |
| `useDepth` | Enable depth-based testing | `true`, `false` | `false` |
| `testDuration` | Test execution duration (seconds) | Float | `300.0` |
| `resetInterval` | Pose reset test interval | `5.0`, `10.0`, `30.0` | `30.0` |
| `absolutePoseTimeout` | Pose validity timeout | Float | `300.0` |
| `cameraInfoTimeout` | Camera info timeout | Float | `15.0` |
| `useHeadYaw` | Head rotation compensation | `true`, `false` | `true` |

### Test Data Configuration

**Test Landmarks** (`data/arucoLandmarksTest.json`):
Simplified landmark set for controlled testing:
```json
{
  "landmarks": [
    {
      "id": 1,
      "x": 5.0,
      "y": 4.8,
      "z": 0.71
    },
    {
      "id": 2,
      "x": 5.6,
      "y": 7.2,
      "z": 0.71
    },
    {
      "id": 3,
      "x": 2.0,
      "y": 5.4,
      "z": 0.71
    }
  ]
}
```

**Test Topics** (`data/pepperTopicsTest.dat`):
Maps test topic names to ROS topics for robot testing.

**Harness Topics** (`data/harnessTopicsTest.dat`):
Maps topic names for test harness environment.

# Running the Tests

## Physical Robot Tests

### Launch Steps

Follow these steps in separate terminals:

1. **Source the workspace:**
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
   ```

2. **Launch the robot:**
   ```bash
   roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
   ```
   
   > **NOTE:** Ensure IP addresses and network interface are correctly configured for your robot setup.

3. **Launch the robot localization test:**
   ```bash
   cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system robotLocalizationTestLaunchRobot.launch
   ```

## Test Harness Tests

For controlled testing with simulated data:

```bash
cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch cssr_system robotLocalizationTestLaunchHarness.launch
```

The harness test includes:
- Odometry data driver with configurable test patterns
- Camera image driver with test image sequences
- Depth image driver for RGB-D testing
- Camera info driver with test calibration
- Joint state driver for head yaw testing
- Service test client for automated service validation
- Test validation node for accuracy assessment


# Test Overview

The robotLocalization unit test validates three main functional areas:

## Test Categories

### Test A: Communication Functionality Validation
- **Test A_1**: Input Data Processing
  - RGB image processing and ArUco marker detection
  - Depth image processing for trilateration
  - Odometry data integration
  - Camera calibration parameter handling
- **Test A_2**: Output Data Generation  
  - Pose publication at expected rates (1-10 Hz)
  - Marker image publication with detection annotations
  - Service response validation

### Test B: Computation Functionality Validation
- **Test B_1**: Triangulation Algorithm Testing (RGB-only)
  - Position accuracy validation (±0.2m target)
  - Orientation accuracy validation (±5° target)
  - Geometric constraint validation
  - Multi-solution scoring system testing
- **Test B_2**: Trilateration Algorithm Testing (RGB-D)
  - Position accuracy with depth data (±0.5m target)
  - Orientation accuracy with depth (±8° target)
  - Circle intersection algorithm validation

### Test C: Configuration Functionality Validation
- **Test C_1**: Parameter Impact Testing
  - Verbose mode toggle functionality
  - Depth usage mode switching
  - Reset interval behavior validation
  - Head yaw compensation testing
- **Test C_2**: Landmark and Camera Configuration Testing
  - Marker position configuration validation
  - Camera calibration parameter testing
  - Invalid configuration handling
  - Fallback mechanism testing

# Test Execution Details

## Unit Test Features

The test system includes special unit test functionality:

### Startup Validation
- Copyright message display verification
- Subscription confirmation for all required topics
- Camera intrinsic parameter validation
- Landmark configuration loading verification

### Runtime Monitoring
- Heartbeat messages every 10 seconds: `"robotLocalization: running"`
- Uptime tracking and status reporting
- Data availability confirmation (images, odometry, etc.)
- Current pose status reporting

### Communication Testing
- First message reception logging for all sensor inputs
- Topic publication rate monitoring
- Service availability and response validation
- Transform publication verification

## Test Scenarios

### Localization Test
- Standard RGB-only triangulation
- Expected accuracy: ±0.2m position, ±5° orientation
- Minimum 3 markers required
- Tests geometric constraint validation

### Depth Test  
- RGB-D trilateration algorithm
- Expected accuracy: ±0.5m position, ±8° orientation
- Depth measurement integration
- Circle intersection validation

### Configuration Robustness Test
- Parameter change impact assessment
- Invalid configuration handling
- Fallback mechanism validation
- Recovery behavior testing

# Expected Test Results

## Performance Metrics

### Success Criteria
- **Pose Publication Rate**: 1-10 Hz sustained
- **Position Accuracy (RGB)**: ±0.2m under good conditions
- **Position Accuracy (RGB-D)**: ±0.5m with depth data
- **Orientation Accuracy**: ±5° (RGB), ±8° (RGB-D)
- **Landmark Detection Rate**: >90% when 3+ markers visible
- **System Latency**: <500ms from image to pose

### Test Output Format

Results are logged to files for analysis:
- Robot tests: `~/workspace/pepper_rob_ws/src/cssr4africa/unit_tests/robotLocalizationTest/test_data/robotLocalizationTestOutput.dat`
- Simulator tests: `~/workspace/pepper_sim_ws/src/cssr4africa/unit_tests/robotLocalizationTest/test_data/robotLocalizationTestOutput.dat`

### Sample Test Report
```
Robot Localization Test Report: robot
====================================
Date: 2025-08-19 14:03:22

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
		Position Accuracy        : PASSED (±0.2m)
		Orientation Accuracy     : PASSED (±5.0°)
	Trilateration Algorithm  
		Position Accuracy        : PASSED (±0.5m)
		Orientation Accuracy     : PASSED (±8.0°)
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
	Position Accuracy (RGB-D): ±0.42m  
	Orientation Accuracy     : ±4.2°
	Landmark Detection Rate  : 94%
	System Latency          : 285ms

Overall Test Result: PASSED
```

# Manual Test Commands

For additional validation and debugging:

```bash
# Monitor pose output
rostopic echo /robotLocalization/pose

# View marker detection
rosrun image_view image_view image:=/robotLocalization/marker_image

# Test services manually
rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
rosservice call /robotLocalization/reset_pose

# Check performance metrics
rostopic hz /robotLocalization/pose
rostopic hz /camera/color/image_raw

# Monitor node status
rosnode info /robotLocalization
```

# Troubleshooting Test Issues

## Common Test Failures

### Test A Failures (Communication)
- **No image data**: Check camera connection and drivers
- **No odometry**: Verify robot connection and naoqi_driver
- **Camera info timeout**: Check RealSense camera info publication
- **Low publication rate**: System performance or processing issues

### Test B Failures (Computation)  
- **Poor position accuracy**: Check marker placement and camera calibration
- **No markers detected**: Verify lighting conditions and marker visibility
- **Triangulation failures**: Ensure non-collinear marker configurations
- **High orientation errors**: Calibrate camera orientation and head yaw

### Test C Failures (Configuration)
- **Parameter not applied**: Check JSON configuration file syntax
- **Invalid config not handled**: Configuration validation logic issues
- **Fallback not triggered**: Camera info timeout or file access issues

## Debug Commands

```bash
# Check test configuration
rosparam get /robotLocalization/

# Monitor test heartbeat
rosout | grep "robotLocalization: running"

# Verify marker setup  
rosrun image_view image_view image:=/robotLocalization/marker_image

# Check transform tree
rosrun tf view_frames
```

# Test Environment Setup

## ArUco Marker Placement

For consistent test results:
- Use DICT_4X4_100 markers printed at consistent size
- Place at heights specified in test landmark configuration
- Ensure adequate lighting without glare or shadows
- Maintain clear line-of-sight from robot operating positions
- Test marker visibility with camera before running tests

## Controlled Testing Tips

- Run tests in consistent lighting conditions
- Minimize environmental changes during test execution
- Use stable network connections for robot communication

> **NOTE:** To fully understand the test procedures, configuration values, and debugging processes, please refer to the [D4.2.4 Robot Localization](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.2.4.pdf). The deliverable provides comprehensive explanations and step-by-step instructions essential for effective testing and troubleshooting.

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: [ioj@alumni.cmu.edu](mailto:ioj@alumni.cmu.edu), [david@vernon.eu](mailto:david@vernon.eu)
- Visit: [www.cssr4africa.org](http://www.cssr4africa.org)

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date: 2025-06-23