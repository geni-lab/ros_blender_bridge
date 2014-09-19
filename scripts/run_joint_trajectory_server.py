#!/usr/bin/env python
import rospy
from ros_blender_bridge import BlenderJointTrajectoryServer

rospy.init_node('blender_joint_trajectory_server')
controller = BlenderJointTrajectoryServer()
rospy.spin()