#!/usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = "James Diprose"

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import yaml
from ros_blender_bridge import BlenderUtils
from threading import Thread

try:
    import bpy
except ImportError:
    print('bpy imported outside of blender')


class BlenderJointStatePublisher(Thread):
    def __init__(self):
        super(BlenderJointStatePublisher, self).__init__()

        self.rate = rospy.Rate(10)
        self.path = rospy.get_param('blender_target_controllers')
        self.armature_name = rospy.get_param('armature_name')

        self.joint_states = JointState()
        self.joint_states.name = BlenderJointStatePublisher.get_joint_names_from_file(self.path)
        self.joint_state_pub = rospy.Publisher('desired_joint_states', JointState, queue_size=10)

    @staticmethod
    def get_joint_names_from_file(path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        joint_names = []

        for ctrl_name, properties in list(config.items()):
            for joint_name, joint_config in list(properties['joints'].items()):
                joint_names.append(joint_name)

        return joint_names

    def run(self):
        BlenderUtils.wait_until_loaded()

        while not rospy.is_shutdown():
            armature = BlenderUtils.get_armature(self.armature_name)
            joint_positions = []

            for name in self.joint_states.name:
                pose_bone = BlenderUtils.get_pose_bone(armature, name)
                angle = BlenderUtils.get_bone_rotation(pose_bone)
                joint_positions.append(angle)

            self.joint_states.position = joint_positions
            self.joint_states.header.stamp = rospy.Time().now()
            self.joint_state_pub.publish(self.joint_states)

            self.rate.sleep()

