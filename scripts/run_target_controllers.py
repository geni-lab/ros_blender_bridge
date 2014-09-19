#!/usr/bin/env python
import rospy
from ros_blender_bridge import ArmatureController, BlenderUtils

rospy.init_node('blender_target_controllers')

controller = ArmatureController()
controller.start()
rospy.spin()