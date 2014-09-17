#!/usr/bin/env python
import rospy
from ros_blender_bridge import ArmatureController

rospy.init_node('bge_armature_controller')
controller = ArmatureController()
controller.run()
rospy.spin()