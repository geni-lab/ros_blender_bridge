#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from bge import logic
from mathutils import Vector
import yaml


class BgeJointStatePublisher(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.armature_config = rospy.get_param('armature_config')
        self.base_frame = rospy.get_param('base_frame', 'base_link')

        self.config = BgeJointStatePublisher.parse_config(self.armature_config)

        self.joint_state = JointState()
        self.joint_state_pub = rospy.Publisher('desired_joint_state', JointState)

    @staticmethod
    def parse_config(self, path):
        with open(path, 'r') as file:
            config = yaml.load(file)

        return config

    @staticmethod
    def get_bone_angle(self, bone):
        axis = self.config[bone.name]['axis']

        if axis == 'x':
            return bone.joint_rotation[0]
        elif axis == 'y':
            return bone.joint_rotation[1]
        else:
            return bone.joint_rotation[2]

    def publish_joint_state(self):
        objects = logic.getCurrentScene().objects
        armature = objects[self.armature_name]

        joint_names = []
        joint_positions = []

        for bone in armature.channels:
            if bone.name.endswith('_joint'):
                position = BgeJointStatePublisher.get_bone_angle(bone)

                joint_names.append(bone.name)
                joint_positions.append(position)

        self.joint_state.name = joint_names
        self.joint_state.position = joint_positions
        self.joint_state.header.stamp = rospy.Time().now()
        self.joint_state.header.frame_id = self.base_frame

        self.joint_state_pub.publish(self.joint_state)

