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
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty, EmptyResponse
import yaml
from ros_blender_bridge import BlenderUtils

try:
    import bpy
except ImportError:
    print('bpy imported outside of blender')


class Target(object):
    def __init__(self, controller_name, bone_name, target_topic, default_position):
        self.controller_name = controller_name
        self.bone_name = bone_name
        self.target_topic = target_topic
        self.point = None
        self.default_position = default_position
        rospy.Subscriber(self.controller_name + '/' + self.target_topic, PointStamped, self.__update_target)
        rospy.Service(self.controller_name + '/reset', Empty, self.reset)
        rospy.loginfo('creating {0} with default position: {1}'.format(self.target_path, self.default_position))

    def __repr__(self):
        return self.target_topic

    def reset(self, req):
        rospy.loginfo('reset {0} to default: {1}'.format(self.target_topic, self.default_position))
        if self.is_initialized():
            self.point = [self.default_position.x, self.default_position.y, self.default_position.z]
        return EmptyResponse()

    def is_initialized(self):
        return self.point is not None and self.default_position is not None

    @staticmethod
    def to_blender_point(ros_point):
        return [-ros_point.y, ros_point.x, ros_point.z]

    def __update_target(self, msg):
        self.point = Target.to_blender_point(msg.point)
        rospy.loginfo('update {0} to : {1}'.format(self.target_path, self.point))


class BlenderTargetListener(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.targets = self.get_targets_from_file()
        rospy.loginfo('targets loaded: ' + str(self.targets))

    def get_targets_from_file(self, path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        armature = BlenderUtils.get_armature(self.armature_name)
        targets = []

        for controller_name, properties in list(config.items()):
            frame_id = properties['frame_id']
            bone_name = properties['target_bone']
            target_topic = properties['target_topic']

            pose_bone = BlenderUtils.get_pose_bone(armature, bone_name)
            default_position = BlenderUtils.get_bone_location(armature, pose_bone, frame_id)

            target = Target(controller_name, bone_name, target_topic, Point(x=default_position[0], y=default_position[1], z=default_position[2]))
            targets.append(target)

        return targets

    def update(self):
        armature = BlenderUtils.get_armature(self.armature_name)
        BlenderUtils.enable_pose_mode(armature)

        for target in self.targets:
            if target.is_initialized():
                frame_id = target.frame_id
                pose_bone = BlenderUtils.get_pose_bone(armature, target.bone_name)
                BlenderUtils.set_bone_location(pose_bone, target.point, frame_id)

        # Update Scene
        bpy.context.scene.update()

