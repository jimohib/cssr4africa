# Active Head Scanning for Marker-Based Localization

## Overview

This document describes the implementation of active head scanning for ArUco marker-based absolute localization in the CSSR4Africa robot system. The enhancement enables the robot to actively search for markers in its environment when insufficient markers are visible in its current field of view.

## Problem Statement

The original localization system required 3 ArUco markers to be visible simultaneously in the robot's camera field of view for absolute localization. When fewer than 3 markers were detected, the system would fail to compute an absolute pose, relying solely on odometry which accumulates drift over time.

## Solution

The active head scanning system enables the robot to:
1. **Detect markers in current view** and store them in memory
2. **Scan additional head positions** when < 3 markers are found
3. **Combine markers from different views** to achieve 3 marker requirement
4. **Account for head yaw differences** when computing angles between markers

## Key Features

### 1. Marker Memory System
- Stores detected markers with metadata:
  - Marker ID
  - Corner coordinates in image frame
  - Head yaw angle when detected
  - Detection timestamp
  - Pre-computed center point
- Automatically cleans up old markers (configurable timeout)
- Updates existing markers when re-detected

### 2. Head Scanning Strategy
- Default scan positions: [-60°, -30°, 0°, 30°, 60°] (configurable)
- Scans only when needed (< 3 markers in current view)
- Returns head to initial position after scanning
- Respects robot joint limits for safety

### 3. Marker Selection
Intelligently selects best 3 markers from memory based on:
- **Recency**: Newer detections weighted higher
- **Image centrality**: Markers closer to image center preferred
- **Landmark validity**: Only known landmarks considered

### 4. Coordinate Transformation
- Accounts for head yaw differences when computing viewing angles
- Transforms pixel coordinates to world frame using stored head yaw
- Maintains geometric accuracy across different head positions

## Implementation Details

### New Data Structures

```cpp
struct DetectedMarker {
    int id;                              // ArUco marker ID
    std::vector<cv::Point2f> corners;    // Corner coordinates
    double head_yaw;                     // Head yaw when detected
    ros::Time timestamp;                 // Detection time
    std::pair<double, double> center;    // Pre-computed center
};
```

### Configuration Parameters

Added to `robotLocalizationConfiguration.json`:

```json
{
  "enableActiveScanning": true,        // Enable/disable active scanning
  "scanTimeout": 2.0,                  // Timeout per scan position (seconds)
  "markerMemoryTimeout": 5.0,          // How long to remember markers (seconds)
  "scanPositions": [-1.047, -0.524, 0.0, 0.524, 1.047]  // Scan positions in radians
}
```

### Key Methods

#### `computeAbsolutePoseWithActiveScanning()`
Main entry point that orchestrates the scanning process:
1. Detects markers in current view
2. Initiates scanning if needed
3. Selects best markers
4. Performs triangulation with head yaw compensation
5. Publishes pose and visualization

#### `moveHeadToPosition(double yaw, double pitch)`
Controls head movement via ROS actionlib:
- Clamps angles to safe joint limits
- Uses FollowJointTrajectoryAction
- Waits for movement completion and image stabilization

#### `detectAndStoreMarkers(double current_head_yaw)`
Detects markers and manages memory:
- Runs ArUco detection on current image
- Updates existing markers or adds new ones
- Associates head yaw with each detection

#### `computeAngleWithHeadYaw(marker1, marker2)`
Computes viewing angle between markers accounting for head yaw:
- Converts pixel coordinates to camera angles
- Adds stored head yaw to get world frame angles
- Returns angular difference in degrees

#### `selectBestMarkers(int count)`
Scores and selects optimal markers:
- Filters by landmark validity
- Scores based on recency and centrality
- Returns top N markers

#### `cleanupOldMarkers()`
Removes stale markers from memory based on timeout

### Integration Points

Modified the following callbacks to use active scanning:
- `resetTimerCallback()`: Periodic absolute pose updates
- `resetPoseCallback()`: Manual pose reset service

## Usage

### Enable/Disable Active Scanning

Edit `cssr_system/robotLocalization/config/robotLocalizationConfiguration.json`:

```json
"enableActiveScanning": true   // Set to false to disable
```

### Adjust Scan Positions

Modify the `scanPositions` array (angles in radians):

```json
"scanPositions": [-1.047, -0.524, 0.0, 0.524, 1.047]  // ±60°, ±30°, 0°
```

### Tune Memory Timeout

Adjust how long markers are remembered:

```json
"markerMemoryTimeout": 5.0  // Seconds
```

## Behavior Examples

### Scenario 1: No Markers in Current View
1. Robot detects 0 markers
2. Scans left (-60°, -30°) and finds marker A
3. Scans center (0°) and finds marker B
4. Scans right (30°) and finds marker C
5. Uses markers A, B, C for localization
6. Restores head to initial position

### Scenario 2: 2 Markers in Current View
1. Robot detects markers A and B
2. Stores both with current head yaw
3. Scans to find 1 more marker
4. Finds marker C at -30°
5. Uses all 3 markers for localization

### Scenario 3: 3+ Markers Already Visible
1. Robot detects markers A, B, C, D
2. Immediately proceeds with localization
3. No scanning needed
4. Selects best 3 markers based on scoring

## Files Modified

### Header Files
- `cssr_system/robotLocalization/include/robotLocalization/robotLocalizationInterface.h`
  - Added `DetectedMarker` struct
  - Added member variables for scanning
  - Added method declarations

### Implementation Files
- `cssr_system/robotLocalization/src/robotLocalizationImplementation.cpp`
  - Implemented all active scanning methods
  - Modified callbacks to use new system

### Application Files
- `cssr_system/robotLocalization/src/robotLocalizationApplication.cpp`
  - Added configuration loading for scanning parameters
  - Initialized head control client

### Configuration Files
- `cssr_system/robotLocalization/config/robotLocalizationConfiguration.json`
  - Added scanning parameters

### Data Files
- `cssr_system/robotLocalization/data/pepperTopics.dat`
  - Added HeadController topic mapping

## Dependencies

### ROS Packages
- `actionlib`: For head control action client
- `control_msgs`: FollowJointTrajectoryAction message types
- `trajectory_msgs`: JointTrajectory message types

### Libraries
- OpenCV (with ArUco module)
- Boost (for shared pointers)
- Angles library (angle normalization)

## Testing Recommendations

1. **Single Marker Test**: Place only 1 marker and verify scanning finds more
2. **Scattered Markers Test**: Place markers at different angles requiring head movement
3. **Dense Markers Test**: Place many markers to test selection algorithm
4. **Timeout Test**: Verify old markers are cleaned up properly
5. **Restore Position Test**: Confirm head returns to initial position after scan

## Performance Considerations

- **Scan Duration**: Each scan position takes ~1-2 seconds (configurable)
- **Full Scan Time**: ~5-10 seconds for complete scan of 5 positions
- **Memory Overhead**: Minimal (stores only marker metadata, not images)
- **CPU Usage**: Comparable to original implementation (same ArUco detection)

## Future Enhancements

1. **Adaptive Scanning**: Learn common marker positions and prioritize those
2. **Parallel Processing**: Process multiple head positions concurrently
3. **Incremental Updates**: Update pose as each marker is found
4. **Visual Odometry Integration**: Combine with VO for better tracking during scans
5. **Marker Quality Assessment**: Factor in detection confidence and marker size

## Troubleshooting

### Active Scanning Not Working
- Check `enableActiveScanning` is `true` in config
- Verify head controller action server is running
- Check log for "Head controller connected" message

### Head Not Moving
- Verify HeadController topic is correct in pepperTopics.dat
- Check robot joint limits
- Ensure action server is accessible

### Poor Localization Accuracy
- Increase `markerMemoryTimeout` to keep markers longer
- Adjust scan positions to cover more area
- Check marker placement for good triangulation geometry

## Author

Implementation by: Claude (Anthropic AI)
Date: November 11, 2025
Version: v1.0

## References

- Original localization system: `robotLocalization/ReadMe.md`
- ArUco marker detection: OpenCV ArUco module documentation
- Triangulation algorithm: Circle-circle intersection method
