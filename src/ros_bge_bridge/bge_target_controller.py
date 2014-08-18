import rospy
from geometry_msgs.msg import PointStamped
from bge import logic


class BgeTargetController(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.robot_name = rospy.get_param('robot_name', 'robot')
        self.gaze_target_name = rospy.get_param('gaze_target_name', 'gaze_target')
        self.gaze_target = None

    def start(self):
        rospy.Subscriber(self.robot_name + '/' + self.gaze_target_name, PointStamped, self.update_gaze_target)

    def update_gaze_target(self, msg):
        self.gaze_target = msg.point

    @staticmethod
    def to_bge_point(self, ros_point):
        return [-ros_point.y, ros_point.x, ros_point.z]

    def update(self):
        if self.gaze_target is not None:
            # Set target position
            objects = logic.getCurrentScene().objects
            gaze_target = objects[self.gaze_target_name]
            gaze_target.worldPosition = BgeTargetController.to_bge_point(self.gaze_target.point)

            # Update Armature
            armature = objects[self.armature_name]
            armature.update()

