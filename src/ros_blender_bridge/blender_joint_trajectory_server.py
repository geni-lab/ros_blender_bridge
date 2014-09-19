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
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from ros_blender_bridge.srv import GetJointTrajectory, GetJointTrajectoryRequest, GetJointTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from threading import RLock
from ros_blender_bridge import BlenderUtils

try:
    import bpy
except ImportError:
    print('bpy imported outside of blender')


class BlenderJointTrajectoryServer(object):
    def __init__(self):
        self.armature_name = rospy.get_param('armature_name', 'Armature')
        self.armature_lock = RLock()
        rospy.Service('get_joint_trajectory', GetJointTrajectory, self.get_joint_trajectory_callback)

    def get_joint_trajectory_callback(self, req):
        #TODO: use NLA Python API to blend / loop animations: http://wiki.blender.org/index.php/User:Rking/Doc:2.6/Manual/Animation/Editors/NLA
        success = True
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time().now() #TODO: check header time, might want to be a param

        with self.armature_lock:
            scene = bpy.context.scene
            screen = bpy.ops.screen
            armature = bpy.data.objects[self.armature_name]
            actions = bpy.data.actions
            seconds_per_frame = 1.0 / scene.render.fps

            if req.action_name in actions:
                action = actions[req.action_name]
                start = int(action.frame_range[0])
                end = int(action.frame_range[1])
                joint_names = BlenderUtils.get_joint_names(action)
                prev_position = 0.0

                # Set current action and reset timeline
                armature.animation_data_create()
                armature.animation_data.action = action

                for i in range(start, end):
                    scene.frame_set(frame=i)
                    scene.update()
                    point = JointTrajectoryPoint()

                    for joint_name in joint_names:
                        pose_bone = armature.pose.bones[joint_name]
                        position = BlenderUtils.get_bone_rotation(pose_bone)
                        velocity = abs(prev_position - position) * seconds_per_frame

                        point.positions.append(position)
                        point.velocities.append(velocity)
                        point.time_from_start = rospy.Duration.from_sec(i * seconds_per_frame)
                        msg.points.append(point)
                        prev_position = position

            else:
                rospy.logerr('Action {0} does not exist in this .blend file'.format(req.action_name))
                success = False

        response = GetJointTrajectoryResponse()
        response.success = success
        response.joint_trajectory = msg

        return response
