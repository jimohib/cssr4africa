# RobotNavigation Node Usage Guide

This guide provides concise instructions on how to:

- Install dependencies
- Build the package
- Run the node
- Echo the pose topic
- Use the reset pose service
- Send a goal

---

## Prerequisites

- ROS Installed: Ensure ROS is installed.
- Catkin Workspace: Place the `robotNavigation` package in the catkin workspace at `~/workspace/pepper_rob_ws/src/cssr4Africa/cssr_system/robotNavigation`.

---

## Package Location

The `robotNavigation` package should be located in:

```
~/workspace/pepper_rob_ws/src/cssr4Africa/cssr_system/robotNavigation
```

---

## Step 1: Install Dependencies

Navigate to the catkin workspace and install dependencies:

```bash
cd ~/workspace/pepper_rob_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 2: Build the Package

Build the workspace:

```bash
catkin_make
source devel/setup.bash
```

---

## Step 3: Run the Node

Start the `robotNavigation` node:

```bash
rosrun cssr_system robotNavigation
```

---

## Step 4: Echo the Pose Topic

View the pose data assuming robot localization is running and publishing the current pose:

```bash
rostopic echo /robotNavigation/pose
```

---

## Step 5: Use the Reset Pose Service

Reset the robot's pose:

```bash
rosservice call /robotNavigation/reset_pose 2.0 3.0 90.0
```

---

## Step 6: Send a Goal

Send a goal to the robot:

```bash
rosservice call /robotNavigation/set_goal 3.0 6.6 0.0
```


