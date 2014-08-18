#! /usr/bin/env python
import rospy
from threading import RLock, Thread
import yaml
from ros_pololu_servo.msg import ServoPololu
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sensor_msgs.msg import JointState
import abc


class Joint(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, bge_controller):
        self.name = name
        self.bge_controller = bge_controller

    @abc.abstractmethod
    def set_position(self, position):
        """
        Sets the position of the motor
        :param position: motor position in radians
        :return:
        """
        return


class PololuJoint(Joint):

    def __init__(self, name, bge_controller):
        Joint.__init__(name, bge_controller)
        self.msg = ServoPololu()
        self.msg.name = self.name
        self.motor_pub = rospy.Publisher('cmd_pololu', ServoPololu)

    def set_position(self, position):
        self.msg.angle = position
        self.msg.acceleration = self.bge_controller.acceleration
        self.msg.speed = self.bge_controller.speed
        self.motor_pub.publish(self.msg)

class DynamixelJoint(Joint):

    def __init__(self, name, bge_controller):
        Joint.__init__(name, bge_controller)
        self.motor_pub = rospy.Publisher(self.name + '/command', Float64)

    def set_position(self, position):
        self.motor_pub.publish(position)


class BgeController(object):
    DEFAULT_SPEED = 0.5
    DEFAULT_ACCELERATION = 0.1

    def __init__(self, name):
        self.name = name
        self.joints = []
        self.active = True
        self.speed = BgeController.DEFAULT_SPEED
        self.acceleration = BgeController.DEFAULT_ACCELERATION

        rospy.Service(self.name + '/enable', Empty, self.enable)
        rospy.Service(self.name + '/disable', Empty, self.disable)
        rospy.Service(self.name + '/set_speed', SetSpeed, self.set_speed)
        rospy.Service(self.name + '/set_accelaration', SetAcceleration, self.set_acceleration)

    def add_joint(self, joint):
        self.joints.append(joint)

    def enable(self):
        self.active = True

    def disable(self):
        self.active = False

    def set_speed(self, req):
        self.speed = req.speed
        return SetSpeedResponse()

    def set_acceleration(self, req):
        self.acceleration = req.acceleration
        return SetAccelerationResponse()

    def is_active(self):
        return self.active


class BgeJointController(Thread):
    def __init__(self):
        rospy.init_node('bge_joint_controller')
        self.rate = rospy.Rate(rospy.get_param('~rate', 10.0))
        self.config = rospy.get_param('~bge_joint_controller_config')
        self.controllers = self.parse_config(self.config)

        self.joint_state_lock = RLock()
        self.joint_state = JointState()
        rospy.Subcriber('desired_joint_state', JointState, self.update_desired_joint_state)

    @staticmethod
    def parse_config(self, path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        controllers = []

        for ctrl_name, joints in list(config.items()):
            bge_controller = BgeController(ctrl_name)
            controllers.append(bge_controller)
            for joint_name, motor in list(joints.items()):
                motor_type = motor['motor']

                if motor_type == 'pololu':
                    joint = PololuJoint(joint_name, bge_controller)
                elif motor_type == 'dynamixel':
                    joint = DynamixelJoint(joint_name, bge_controller)
                else:
                    rospy.logerr('{0} joint type does not exist'.format(motor_type))

                bge_controller.add_joint(joint)

        return controllers

    def update_desired_joint_state(self, msg):
        with self.joint_state_lock:
            self.joint_state = msg

    def get_desired_position(self, joint):
        index = self.joint_state.name.index(joint.name)
        position = self.joint_state.position[index]
        return position

    def run(self):
        while not rospy.is_shutdown():
            for controller in self.controllers:
                if controller.is_active():
                    for joint in controller.joints:
                        position = self.get_desired_position(joint)
                        joint.set_position(position)

            self.rate.sleep()

if __name__ == '__main__':
    controller = BgeJointController()
    controller.start()
    rospy.spin()