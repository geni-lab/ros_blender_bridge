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
from threading import RLock, Thread
import yaml
import abc
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from ros_blender_bridge.srv import SetSpeed, SetAcceleration, SetSpeedResponse, SetAccelerationResponse
from std_msgs.msg import Bool, Empty as EmptyMsg
import importlib


class Joint(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.speed = 0.0
        self.acceleration = 0.0

    @abc.abstractmethod
    def set_position(self, position):
        """
        Sets the position of the motor
        :param position: motor position in radians
        :return:
        """
        return

    @abc.abstractmethod
    def set_speed(self, speed):
        """
        Sets the speed of the motor
        :param speed: motor speed
        :return:
        """
        return

    @abc.abstractmethod
    def set_acceleration(self, acceleration):
        """
        Sets the acceleration of the motor
        :param acceleration: motor acceleration
        :return:
        """
        return


class TargetController(object):
    def __init__(self, name):
        self.name = name
        self.joints = []
        self.active = True
        self.lock = RLock()

        rospy.Service(self.name + '/enable', Empty, self.enable)
        rospy.Service(self.name + '/disable', Empty, self.disable)
        rospy.Service(self.name + '/set_speed', SetSpeed, self.set_speed)
        rospy.Service(self.name + '/set_acceleration', SetAcceleration, self.set_acceleration)
        self.joints_stopped_pub = rospy.Publisher(self.name + '/joints_stopped', EmptyMsg, queue_size=1)
        self.joints_stopped = False

    def add_joint(self, joint):
        self.joints.append(joint)

    def enable(self, req):
        with self.lock:
            self.active = True
            self.joints_stopped = False
            return EmptyResponse()

    def disable(self, req):
        with self.lock:
            self.active = False
            return EmptyResponse()

    def publish_joints_stopped(self):
        self.joints_stopped_pub.publish(EmptyMsg())

    def set_speed(self, req):
        with self.lock:
            for joint in self.joints:
                joint.set_speed(req.speed)

            return SetSpeedResponse()

    def set_acceleration(self, req):
        with self.lock:
            for joint in self.joints:
                joint.set_acceleration(req.acceleration)

            return SetAccelerationResponse()

    def is_active(self):
        with self.lock:
            return self.active


class ArmatureController(Thread):
    def __init__(self):
        super(ArmatureController, self).__init__()

        self.rate = rospy.Rate(rospy.get_param('~rate', 10.0))
        self.config = rospy.get_param('blender_target_controllers')
        self.controller_list = self.get_controllers_from_file(self.config)

        self.desired_joint_states = JointState()

        self.lock = RLock()
        self.joint_states = JointState()
        rospy.Subscriber('desired_joint_states', JointState, self.update_desired_joint_states)
        rospy.Subscriber('joint_states', JointState, self.update_joint_states)

    @staticmethod
    def get_controllers_from_file(path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        controller_list = []

        for ctrl_name, properties in list(config.items()):
            target_controller = TargetController(ctrl_name)

            default_speed = properties['default_speed']
            default_acceleration = properties['default_acceleration']

            for joint_name, joint_config in list(properties['joints'].items()):
                module_name = joint_config['module']
                class_name = joint_config['class']
                module = importlib.import_module(module_name)
                joint = getattr(module, class_name)(joint_name)
                joint.set_speed(default_speed)
                joint.set_acceleration(default_acceleration)
                target_controller.add_joint(joint)

            controller_list.append(target_controller)

        return controller_list

    def update_desired_joint_states(self, msg):
        with self.lock:
            self.desired_joint_states = msg

    def update_joint_states(self, msg):
        with self.lock:
            self.joint_states = msg

    def get_desired_position(self, joint):
        index = self.desired_joint_states.name.index(joint.joint_name) #TODO: check if received joint state
        position = self.desired_joint_states.position[index]
        return position

    def get_current_position(self, joint):
        index = self.joint_states.name.index(joint.joint_name)
        position = self.joint_states.position[index]
        return position

    def run(self):
        rospy.wait_for_message('/desired_joint_states', JointState)
        rospy.wait_for_message('/joint_states', JointState)

        while not rospy.is_shutdown():
            with self.lock:
                for controller in self.controller_list:
                    if controller.is_active():

                        num_joints_stopped = 0

                        for joint in controller.joints:
                            desired_position = self.get_desired_position(joint)
                            current_position = self.get_current_position(joint)
                            difference = abs(current_position - desired_position)

                            joint.set_position(desired_position)

                            if difference < 0.0174532925:
                                #rospy.loginfo('name: {0}, difference: {1}'.format(joint.joint_name, str(difference)))
                                num_joints_stopped += 1

                        if controller.joints_stopped is False and num_joints_stopped == len(controller.joints):
                            controller.joints_stopped = True
                            controller.publish_joints_stopped()

            self.rate.sleep()
