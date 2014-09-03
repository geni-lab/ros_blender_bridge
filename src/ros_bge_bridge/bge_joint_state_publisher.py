#!/usr/bin/env python
# Copyright (c) 2014, Jamie Diprose
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
# * Neither the name of the {organization} nor the names of its
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
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
import yaml
from ros_bge_bridge import JointAxesParser

try:
    from bge import logic
    from mathutils import Vector
except ImportError:
    print('Not being executed from Blender Game Engine')


class BgeJointStatePublisher(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.joint_axes_yaml = rospy.get_param('joint_axes_yaml')
        self.base_frame = rospy.get_param('base_frame', 'base_link')

        self.config = JointAxesParser(self.joint_axes_yaml)

        self.joint_state = JointState()
        self.joint_state_pub = rospy.Publisher('desired_joint_state', JointState, queue_size=10)

    def publish_joint_state(self):
        objects = logic.getCurrentScene().objects
        armature = objects[self.armature_name]

        joint_names = []
        joint_positions = []

        for bone in armature.channels:
            if bone.name.endswith('_joint'):
                position = self.config.get_bone_angle(bone)

                #print("joint_rotation: " + str(bone.joint_rotation))

                joint_names.append(bone.name)
                joint_positions.append(position)

        self.joint_state.name = joint_names
        self.joint_state.position = joint_positions
        self.joint_state.header.stamp = rospy.Time().now()
        self.joint_state.header.frame_id = self.base_frame

        self.joint_state_pub.publish(self.joint_state)

