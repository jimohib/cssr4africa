#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def reset_head():
    rospy.init_node('reset_pepper_head')

    pub = rospy.Publisher(
        '/pepper_dcm/Head_controller/command',
        JointTrajectory,
        queue_size=1
    )

    rospy.sleep(1)  # wait for publisher to register

    # Define trajectory
    traj = JointTrajectory()
    traj.joint_names = ['HeadYaw', 'HeadPitch']

    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0]  # [yaw, pitch]
    point.time_from_start = rospy.Duration(1.0)  # 1 second

    traj.points.append(point)

    # Send command
    pub.publish(traj)
    rospy.loginfo("Published head reset command (HeadYaw=0, HeadPitch=0)")

if __name__ == '__main__':
    try:
        reset_head()
    except rospy.ROSInterruptException:
        pass
