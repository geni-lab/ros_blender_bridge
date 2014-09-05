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
from threading import RLock, Thread
import yaml
import abc
from ros_pololu_servo.msg import MotorStateList, MotorCommand
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sensor_msgs.msg import JointState
from ros_bge_bridge.srv import SetSpeed, SetAcceleration, SetSpeedResponse, SetAccelerationResponse
import dynamixel_controllers
from std_msgs.msg import Bool

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


class PololuJoint(Joint):
    def __init__(self, joint_name):
        Joint.__init__(self, joint_name)
        self.msg = MotorCommand()
        self.msg.joint_name = self.joint_name
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)

    def set_position(self, position):
        self.msg.position = position
        self.msg.acceleration = self.acceleration
        self.msg.speed = self.speed
        self.motor_pub.publish(self.msg)

    def set_speed(self, speed):
        self.speed = speed

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration


class DynamixelJoint(Joint):
    def __init__(self, joint_name):
        Joint.__init__(self, joint_name)
        self.motor_pub = rospy.Publisher(self.joint_name + '/command', Float64, queue_size=10)
        self.set_speed_srv = rospy.ServiceProxy(self.joint_name + '/set_speed', dynamixel_controllers.srv.SetSpeed)

    def set_position(self, position):
        self.motor_pub.publish(position)

    def set_speed(self, speed):
        self.set_speed_srv(speed)

    def set_acceleration(self, acceleration):
        pass


class BgeController(object):
    def __init__(self, name):
        self.name = name
        self.joints = []
        self.active = True

        rospy.Service(self.name + '/enable', Empty, self.enable)
        rospy.Service(self.name + '/disable', Empty, self.disable)
        rospy.Service(self.name + '/set_speed', SetSpeed, self.set_speed)
        rospy.Service(self.name + '/set_acceleration', SetAcceleration, self.set_acceleration)
        self.target_reached_pub = rospy.Publisher(self.name + '/target_reached', Bool)
        self.target_reached = False

    def add_joint(self, joint):
        self.joints.append(joint)

    def enable(self, req):
        self.active = True
        self.set_target_reached(False)
        return EmptyResponse()

    def disable(self, req):
        self.active = False
        self.set_target_reached(False)
        return EmptyResponse()

    def set_target_reached(self, target_reached):
        if target_reached != self.target_reached:
            self.target_reached = target_reached
            self.publish_target_reached()

    def publish_target_reached(self):
        self.target_reached_pub.publish(self.target_reached)

    def set_speed(self, req):
        for joint in self.joints:
            joint.set_speed(req.speed)

        return SetSpeedResponse()

    def set_acceleration(self, req):
        for joint in self.joints:
            joint.set_acceleration(req.acceleration)

        return SetAccelerationResponse()

    def is_active(self):
        return self.active


class BgeArmatureController(Thread):
    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param('~rate', 10.0))
        self.config = rospy.get_param('~bge_controllers_yaml')
        self.controllers = self.parse_config(self.config)

        self.desired_joint_state_lock = RLock()
        self.desired_joint_state = JointState()

        self.joint_state_lock = RLock()
        self.joint_state = JointState()
        rospy.Subscriber('desired_joint_states', JointState, self.update_desired_joint_state)
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)

    @staticmethod
    def parse_config(path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        controllers = []

        for ctrl_name, properties in list(config.items()):
            default_speed = properties['default_speed']
            default_acceleration = properties['default_acceleration']
            bge_controller = BgeController(ctrl_name)
            controllers.append(bge_controller)

            for joint_name, motor in list(properties['joints'].items()):
                motor_type = motor['motor']

                if motor_type == 'pololu':
                    joint = PololuJoint(joint_name)
                    joint.set_speed(default_speed)
                    joint.set_acceleration(default_acceleration)
                elif motor_type == 'dynamixel':
                    joint = DynamixelJoint(joint_name)
                    joint.set_speed(default_speed)
                else:
                    rospy.logerr('{0} joint type does not exist'.format(motor_type))

                bge_controller.add_joint(joint)

        return controllers

    def update_desired_joint_state(self, msg):
        with self.desired_joint_state_lock:
            self.desired_joint_state = msg

    def update_joint_state(self, msg):
        with self.joint_state_lock:
            self.joint_state = msg

    def get_desired_position(self, joint):
        index = self.desired_joint_state.name.index(joint.joint_name) #TODO: check if received joint state
        position = self.desired_joint_state.position[index]
        return position

    def get_current_position(self, joint):
        index = self.joint_state.name.index(joint.joint_name)
        position = self.joint_state.position[index]
        return position

    def run(self):
        rospy.wait_for_message('/desired_joint_states', JointState)
        rospy.wait_for_message('/joint_states', JointState)

        while not rospy.is_shutdown():
            for ctrlr in self.controllers:
                if ctrlr.is_active():
                    target_reached = 0

                    for joint in ctrlr.joints:
                        desired_position = self.get_desired_position(joint)
                        current_position = self.get_current_position(joint)
                        difference = abs(current_position - desired_position)

                        joint.set_position(desired_position)
                        rospy.loginfo('name: {0}, difference: {1}'.format(joint.joint_name, str(difference)))
                        if difference < 0.0174532925:

                            target_reached += 1

                    if target_reached == len(ctrlr.joints):
                        ctrlr.set_target_reached(True)
                    else:
                        ctrlr.set_target_reached(False)

            self.rate.sleep()
