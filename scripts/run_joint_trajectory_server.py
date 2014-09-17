#!/usr/bin/env python
import rospy
from ros_blender_bridge import BpyJointTrajectoryServer

rospy.init_node('bpy_joint_trajectory_server')
controller = BpyJointTrajectoryServer()
rospy.spin()