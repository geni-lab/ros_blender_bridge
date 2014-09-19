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
from threading import Thread

try:
    import bpy
    from mathutils import Vector, Matrix
except ImportError:
    print('bpy imported outside of blender')


class Target(object):
    def __init__(self, controller_name, frame_id, bone_name, target_topic, default_position):
        self.controller_name = controller_name
        self.frame_id = frame_id
        self.bone_name = bone_name
        self.target_topic = target_topic
        self.default_position = default_position
        self.full_target_topic = self.controller_name + '/' + self.target_topic
        self.armature_name = rospy.get_param('armature_name', 'Armature')

        rospy.Subscriber(self.full_target_topic, PointStamped, self.__update_target)
        rospy.Service(self.controller_name + '/reset', Empty, self.reset)
        rospy.loginfo('creating {0} with default position: {1}'.format(self.full_target_topic, self.default_position))

    def __repr__(self):
        return self.target_topic

    def reset(self, req):
        rospy.loginfo('reset {0} to default: {1}'.format(self.target_topic, self.default_position))
        if self.is_initialized():
            self.set_position(self.default_position)
        return EmptyResponse()

    def is_initialized(self):
        return self.point is not None and self.default_position is not None

    def set_position(self, position):
        armature = BlenderUtils.get_armature(self.armature_name)
        pose_bone = BlenderUtils.get_pose_bone(armature, self.bone_name)
        BlenderUtils.set_bone_world_location(pose_bone, position)
        bpy.context.scene.update()

    def __update_target(self, msg):
        armature = BlenderUtils.get_armature(self.armature_name)
        origin_bone = BlenderUtils.get_pose_bone(armature, self.frame_id)

        point = BlenderUtils.to_blender_vector(msg.point)
        origin_world = BlenderUtils.get_bone_world_location(origin_bone)
        world_position = origin_world + point

        self.set_position(world_position)
        rospy.loginfo('update {0} to : {1}'.format(self.full_target_topic, msg.point))


class BlenderTargetListener():
    def __init__(self):
        BlenderUtils.wait_until_loaded()
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.path = rospy.get_param('blender_target_controllers')
        self.targets = self.get_targets_from_file(self.path)
        rospy.loginfo('targets loaded and initialized: ' + str(self.targets) + ". listening...")

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
            default_position = BlenderUtils.get_bone_world_location(pose_bone)

            target = Target(controller_name, frame_id, bone_name, target_topic, default_position)
            target.set_position(default_position)
            targets.append(target)

        return targets



