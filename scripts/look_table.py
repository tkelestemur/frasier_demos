#!/usr/bin/env python

import actionlib
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


rospy.init_node('look_table')

# initialize action client
cli = actionlib.SimpleActionClient(
    '/hsrb/impedance_control/follow_joint_trajectory', FollowJointTrajectoryAction)

head_cli = actionlib.SimpleActionClient(
    '/hsrb/head_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)


# wait for the action server to establish connection
cli.wait_for_server()
head_cli.wait_for_server()

traj_goal = FollowJointTrajectoryGoal()
traj = JointTrajectory()

traj.joint_names = ["odom_x", "odom_y", "odom_t", "arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

p = JointTrajectoryPoint()
p.positions = [0.0, 0.0, 0.0, 0.69, -2.60, 0.0, -0.54, 0.0]
p.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
p.time_from_start = rospy.Time(3)

traj.points = [p]
traj_goal.trajectory = traj

cli.send_goal(traj_goal)

cli.wait_for_result()

head_traj_goal = FollowJointTrajectoryGoal()
head_traj = JointTrajectory()

head_traj.joint_names = ["head_pan_joint", "head_tilt_joint"]

h_p = JointTrajectoryPoint()
h_p.positions = [0.0, -0.50]
h_p.velocities = [0.0, 0.0]
h_p.time_from_start = rospy.Time(1)

head_traj.points = [h_p]
head_traj_goal.trajectory = head_traj

head_cli.send_goal(head_traj_goal)
head_cli.wait_for_result()



