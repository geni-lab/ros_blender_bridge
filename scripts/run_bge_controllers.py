#!/usr/bin/env python
import rospy
from ros_bge_bridge import BgeArmatureController

rospy.init_node('bge_armature_controller')
controller = BgeArmatureController()
controller.run()
rospy.spin()