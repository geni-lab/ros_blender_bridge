#!/usr/bin/env python
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

