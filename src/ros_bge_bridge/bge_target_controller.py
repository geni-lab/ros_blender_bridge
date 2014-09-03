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
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty, EmptyResponse

try:
    from bge import logic
except ImportError:
    print('Not being executed from Blender Game Engine')


class Target(object):
    def __init__(self, robot_name, target_name, default_position):
        self.robot_name = robot_name
        self.target_name = target_name
        self.target_path = robot_name + '/' + target_name
        self.point = None
        self.default_position = default_position
        rospy.Subscriber(self.target_path, PointStamped, self.__update_target)
        rospy.Service(self.target_path + '/reset', Empty, self.reset)
        rospy.loginfo('creating {0} with default position: {1}'.format(self.target_path, self.default_position))

    def __repr__(self):
        return self.target_name

    def reset(self, req):
        rospy.loginfo('reset {0} to default: {1}'.format(self.target_path, self.default_position))
        if self.is_initialized():
            self.point = [self.default_position.x, self.default_position.y, self.default_position.z]
        return EmptyResponse()

    def is_initialized(self):
        return self.point is not None and self.default_position is not None

    @staticmethod
    def to_bge_point(ros_point):
        return [-ros_point.y, ros_point.x, ros_point.z]

    def __update_target(self, msg):
        self.point = Target.to_bge_point(msg.point)
        rospy.loginfo('update {0} to : {1}'.format(self.target_path, self.point))


class BgeTargetController(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.robot_name = rospy.get_param('robot_name', 'robot')
        self.targets = self.load_targets()
        rospy.loginfo('targets: ' + str(self.targets))

    def load_targets(self):
        objects = logic.getCurrentScene().objects
        targets = []

        for entity in objects:
            if entity.name.endswith('_target'):
                default_position = objects[entity.name].worldPosition
                target = Target(self.robot_name, entity.name, Point(x=default_position[0], y=default_position[1], z=default_position[2]))
                targets.append(target)

        return targets

    def update(self):
        objects = logic.getCurrentScene().objects

        for target in self.targets:
            if target.point is not None:
                objects[target.target_name].worldPosition = target.point

        # Update Armature
        armature = objects[self.armature_name]
        armature.update()

