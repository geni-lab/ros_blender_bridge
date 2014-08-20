#!/usr/bin/env python
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
    def __init__(self, joint_name, pololu_name):
        Joint.__init__(self, joint_name)
        self.pololu_name = pololu_name

        self.msg = MotorCommand()
        self.msg.joint_name = self.joint_name
        self.motor_pub = rospy.Publisher(self.pololu_name + '/command', MotorCommand, queue_size=10)

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

    def add_joint(self, joint):
        self.joints.append(joint)

    def enable(self):
        self.active = True

    def disable(self):
        self.active = False

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

        self.joint_state_lock = RLock()
        self.joint_state = JointState()
        rospy.Subscriber('desired_joint_state', JointState, self.update_desired_joint_state)

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
                motor_type = motor['motor']['type']
                motor_namespace = motor['motor']['namespace']

                if motor_type == 'pololu':
                    joint = PololuJoint(joint_name, motor_namespace)
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
        with self.joint_state_lock:
            self.joint_state = msg

    def get_desired_position(self, joint):
        index = self.joint_state.name.index(joint.joint_name)
        position = self.joint_state.position[index]
        return position

    def run(self):
        rospy.wait_for_message('/desired_joint_state', JointState)

        while not rospy.is_shutdown():
            for ctrlr in self.controllers:
                if ctrlr.is_active():
                    for joint in ctrlr.joints:
                        position = self.get_desired_position(joint)
                        joint.set_position(position)

            self.rate.sleep()
