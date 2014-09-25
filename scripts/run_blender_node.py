#!/usr/bin/env python
import rospy
from ros_blender_bridge import BlenderTargetListener, BlenderJointStatePublisher, BlenderUtils

rospy.init_node('blender_node')

armature_name = rospy.get_param("armature_name", "Armature")

listener = BlenderTargetListener()
joint_pub = BlenderJointStatePublisher()
joint_pub.start()

show_gui = rospy.get_param("show_blender_gui", False)

if not show_gui:
    rospy.spin() #Have to spin otherwise the script quits
    
