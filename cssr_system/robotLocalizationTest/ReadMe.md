# Robot Localization Component Unit Tests

This document describes the unit testing procedures for the `robotLocalization` component in accordance with the CSSR4Africa System Integration and Quality Assurance Manual.

## Overview

The `robotLocalization` component provides visual landmark-based pose estimation for autonomous robots. These unit tests validate the communication, computation, and configuration functionality of the component across three different testing environments.

## Test Launch Files

### 1. robotLocalizationLaunchRobot.launch
**Purpose**: Tests component with physical Pepper robot and real sensors.

**Data Sources**:
- Physical RealSense RGB-D camera (`/camera/color/image_raw`, `/camera/depth/image_raw`)
- Robot odometry (`/pepper_dcm/odom`)
- Joint states from robot (`/joint_states`)
- Camera calibration (`/camera/color/camera_info`)

**Data Sinks**:
- Robot pose estimates (`/robotLocalization/pose`)
- Annotated marker images (`/robotLocalization/marker_image`)

**Expected Results**:
- Continuous pose publication at ~1-10 Hz
- Pose accuracy within ±0.1m position, ±5° orientation
- Successful marker detection when 3+ ArUco markers visible
- Automatic pose correction every 5 seconds (configurable)

### 2. robotLocalizationLaunchSimulator.launch
**Purpose**: Tests component in Gazebo simulation environment.

**Data Sources**:
- Simulated RGB-D camera in Gazebo
- Simulated robot odometry
- Simulated joint states
- Generated camera calibration

**Data Sinks**:
- Same as robot test but in simulation coordinates
- Test waypoint navigation validation

**Expected Results**:
- Pose tracking during simulated robot movement
- Consistent localization accuracy across different viewpoints
- Proper handling of simulated sensor noise and delays

### 3. robotLocalizationLaunchTestHarness.launch
**Purpose**: Tests component with controlled test data drivers and result logging stubs.

**Data Sources (Drivers)**:
- Pre-recorded image sequences with known ArUco marker positions
- Synthetic odometry data with known trajectories
- Controlled camera calibration parameters
- Programmed joint state sequences

**Data Sinks (Stubs)**:
- Pose data logger (CSV format)
- Marker image logger (annotated images)
- Service response logger
- Performance metrics logger

**Expected Results**:
- Reproducible pose estimation results
- Service response validation
- Performance metrics within acceptable bounds

## Communication Functionality Validation

### Input Data Processing
The component validates proper handling of:

**RGB Image Data** (`sensor_msgs/Image`):
- **Input**: 1280x720 BGR8 images at 5-30 Hz
- **Expected Output**: ArUco marker detection and pose computation
- **Validation**: Annotated images showing detected markers with IDs

**Depth Image Data** (`sensor_msgs/Image`):
- **Input**: 848x480 depth images (16UC1 or 32FC1) aligned to RGB
- **Expected Output**: Distance measurements for trilateration
- **Validation**: Pose estimates using depth-based algorithm when `use_depth=true`

**Odometry Data** (`nav_msgs/Odometry`):
- **Input**: Robot pose and velocity in odometry frame
- **Expected Output**: Continuous pose updates between absolute corrections
- **Validation**: Smooth pose interpolation, drift correction after landmark detection

**Camera Calibration** (`sensor_msgs/CameraInfo`):
- **Input**: Camera intrinsic matrix (fx, fy, cx, cy)
- **Expected Output**: Accurate angle computations for triangulation
- **Validation**: Geometric consistency in pose estimates

### Output Data Generation
**Pose Publication** (`geometry_msgs/Pose2D`):
- **Format**: x (meters), y (meters), theta (degrees)
- **Rate**: 1-10 Hz depending on odometry input rate
- **Validation**: Compare against known landmark positions and expected robot trajectory

**Marker Image Publication** (`sensor_msgs/Image`):
- **Format**: Annotated RGB images with detected ArUco markers outlined
- **Rate**: Same as input image rate when markers detected
- **Validation**: Visual confirmation of marker detection accuracy

## Computation Functionality Validation

### Triangulation Algorithm Testing
**Test Scenario**: Robot positioned at known locations with 3+ visible markers

**Input Data**:
- RGB images containing ArUco markers at IDs [0, 1, 2, 3, 4]
- Known landmark positions: (1.0, 2.0), (3.0, 2.0), (2.0, 4.0), etc.
- Camera intrinsics: fx=525.0, fy=525.0, cx=319.5, cy=239.5

**Expected Output**:
- Position accuracy: ±0.1m for distances <5m, ±0.2m for distances 5-10m
- Orientation accuracy: ±5° under good lighting, ±10° under challenging conditions
- Algorithm should reject degenerate configurations (collinear markers, extreme angles)

**Validation Method**:
```bash
# Set known robot pose
rosservice call /robotLocalization/set_pose 2.0 3.0 0.0

# Wait for automatic reset from landmarks
# Check pose output matches expected position within tolerance
rostopic echo /robotLocalization/pose
```

### Trilateration Algorithm Testing (Depth Mode)
**Test Scenario**: Same positions as triangulation but with depth data enabled

**Input Data**:
- RGB images + aligned depth images
- Distance measurements to landmarks (from depth camera)

**Expected Output**:
- Improved accuracy: ±0.05m position, ±3° orientation
- Faster convergence (requires only 3 markers vs complex triangulation)
- Better performance in challenging lighting conditions

**Validation Method**:
```bash
# Enable depth mode
rosparam set /robotLocalization/use_depth true

# Reset and compare results
rosservice call /robotLocalization/reset_pose
```

### Sensor Fusion Testing
**Test Scenario**: Robot movement with intermittent landmark visibility

**Input Data**:
- Continuous odometry stream during movement
- Periodic landmark detections (every 5-30 seconds)

**Expected Output**:
- Smooth pose updates during movement using odometry
- Drift correction when landmarks reappear
- No pose jumps >0.2m during corrections

**Validation Method**:
Monitor pose continuity during programmed robot movements or test harness replay.

## Configuration Functionality Validation

### Parameter Impact Testing

**1. Verbose Mode Parameter**
```ini
verbose = true/false
```
**Expected Behavior Change**:
- `true`: Detailed diagnostic messages, marker detection logs, pose computation details
- `false`: Minimal output, only errors and warnings

**Validation**:
```bash
# Test verbose output
rosparam set /robotLocalization/verbose true
# Observe increased log output

rosparam set /robotLocalization/verbose false  
# Observe reduced log output
```

**2. Depth Usage Parameter**
```ini
use_depth = true/false
```
**Expected Behavior Change**:
- `true`: Uses trilateration algorithm with depth measurements
- `false`: Uses triangulation algorithm with angle measurements only

**Validation**:
Compare pose accuracy and computation time between modes. Depth mode should show:
- Higher accuracy (±0.05m vs ±0.1m)
- Better performance in poor lighting
- Requires aligned depth images

**3. Reset Interval Parameter**
```ini
reset_interval = 5.0/10.0/30.0 (seconds)
```
**Expected Behavior Change**:
- Shorter intervals: More frequent pose corrections, higher computational load
- Longer intervals: Less drift correction, lower computational load

**Validation**:
```bash
# Test different intervals
rosparam set /robotLocalization/reset_interval 5.0
# Monitor reset frequency in logs

rosparam set /robotLocalization/reset_interval 30.0
# Verify reduced reset frequency
```

**4. Head Yaw Compensation Parameter**
```ini
use_head_yaw = true/false
head_yaw_joint_name = "HeadYaw"
```
**Expected Behavior Change**:
- `true`: Compensates for head rotation in yaw calculations
- `false`: Assumes fixed head orientation

**Validation**:
Move robot head and observe pose estimate stability. With compensation enabled, pose should remain stable during head movement.

**5. Camera Configuration Parameters**
```yaml
camera_info:
  fx: 525.0  # Focal length changes
  fy: 525.0
  cx: 319.5  # Principal point changes  
  cy: 239.5
```
**Expected Behavior Change**:
- Incorrect focal lengths: Systematic position errors
- Incorrect principal point: Angular bias in pose estimates

**Validation**:
Deliberately modify camera parameters and observe pose estimation degradation.

**6. Landmark Configuration Impact**
```yaml
landmarks:
  - id: 0
    x: 1.0    # Position accuracy affects system accuracy
    y: 2.0
    z: 1.225
```
**Expected Behavior Change**:
- Inaccurate landmark positions: Systematic pose errors
- Insufficient landmarks: Reduced robustness
- Poor landmark distribution: Increased geometric dilution of precision

**Validation**:
Test with different landmark configurations:
- Minimum 3 landmarks (triangle)
- 4+ landmarks (redundancy)
- Collinear landmarks (should be rejected)

## Test Execution Commands

### Physical Robot Testing
```bash
# Terminal 1: Launch robot connection
roslaunch unit_tests robotLocalizationLaunchRobot.launch robot_ip:=172.111.29.240

# Terminal 2: Monitor results
rostopic echo /robotLocalization/pose
rosrun image_view image_view image:=/robotLocalization/marker_image

# Terminal 3: Test services
rosservice call /robotLocalization/set_pose 2.0 6.6 0.0
rosservice call /robotLocalization/reset_pose
```

### Simulation Testing
```bash
# Terminal 1: Launch simulation
roslaunch unit_tests robotLocalizationLaunchSimulator.launch gui:=true

# Terminal 2: Monitor and control
rostopic echo /robotLocalization/pose
# Robot will move automatically through test waypoints
```

### Test Harness Testing
```bash
# Terminal 1: Launch test harness
roslaunch unit_tests robotLocalizationLaunchTestHarness.launch test_scenario:=basic_localization

# Results will be logged to:
# - test_results/pose_log_basic_localization.csv
# - test_results/marker_images_basic_localization/
# - test_results/validation_basic_localization.csv
```

## Expected Test Results

### Performance Metrics
- **Pose Publication Rate**: 1-10 Hz
- **Position Accuracy**: ±0.1m (RGB), ±0.05m (RGB-D)
- **Orientation Accuracy**: ±5° (RGB), ±3° (RGB-D)
- **Landmark Detection Rate**: >90% when 3+ markers visible
- **System Latency**: <500ms from image to pose

### Success Criteria
1. **Startup**: Copyright message displayed, all subscriptions confirmed
2. **Communication**: All input topics received, output topics publishing
3. **Computation**: Pose estimates within accuracy tolerances
4. **Configuration**: Parameter changes produce expected behavior
5. **Robustness**: Graceful handling of missing data, invalid configurations
6. **Heartbeat**: "robotLocalization: running" message every 10 seconds

### Failure Indicators
- No pose output after 30 seconds
- Pose accuracy >0.5m consistently
- No marker detection with visible markers
- System crashes or memory leaks
- Parameter changes ignored

## Troubleshooting Test Issues

### Common Test Failures

**1. No Camera Data**
```bash
# Check camera topics
rostopic list | grep camera
rostopic hz /camera/color/image_raw

# Verify camera launch
roslaunch realsense2_camera rs_camera.launch
```

**2. No Landmark Detection**
- Verify ArUco markers use DICT_4X4_100 dictionary
- Check marker placement matches landmarks.yaml
- Ensure adequate lighting and marker visibility
- Verify camera calibration accuracy

**3. Inaccurate Poses**
- Validate landmark position measurements
- Check camera calibration parameters
- Verify coordinate frame transformations
- Test with known reference positions

**4. Service Failures**
```bash
# Check service availability
rosservice list | grep robotLocalization
rosservice info /robotLocalization/set_pose

# Test service manually
rosservice call /robotLocalization/reset_pose
```

## Test Data Requirements

### Test Harness Data Files
- `test_data/test_odometry.csv`: Synthetic odometry trajectories
- `test_data/test_images/`: RGB image sequences with ArUco markers
- `test_data/test_depth/`: Corresponding depth images
- `test_data/test_camera_info.yaml`: Known camera calibration
- `test_data/expected_poses_*.csv`: Ground truth poses for validation

### Landmark Configuration Files
- `config/test_landmarks.yaml`: Test environment marker positions
- `config/test_camera_info.yaml`: Camera parameters for testing
- `data/testHarnessTopics.dat`: Topic remapping for test harness

This comprehensive testing framework ensures the `robotLocalization` component meets all CSSR4Africa quality standards for communication, computation, and configuration functionality.