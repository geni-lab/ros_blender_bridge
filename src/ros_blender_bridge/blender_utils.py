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

try:
    import bpy
    from mathutils import Vector, Matrix
except ImportError:
    print('bpy imported outside of blender')


class BlenderUtils():

    @staticmethod
    def wait_until_loaded():
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo('Waiting for Blender to load')

            if isinstance(bpy.data, bpy.types.BlendData):
                if "Armature" in bpy.data.objects:
                    rospy.loginfo('Loaded!!!')
                    break

            rate.sleep()

    @staticmethod
    def get_armature(armature_name):
        return bpy.data.objects[armature_name]

    @staticmethod
    def get_pose_bone(armature, bone_name):
        return armature.pose.bones[bone_name]

    @staticmethod
    def enable_pose_mode(armature):
        bpy.context.scene.objects.active = armature
        bpy.ops.object.mode_set(mode='POSE')

    @staticmethod
    def get_bone_axis(pose_bone):
        if not pose_bone.lock_ik_x and pose_bone.lock_ik_y and pose_bone.lock_ik_z:
            axis = 'x'
        elif not pose_bone.lock_ik_y and pose_bone.lock_ik_x and pose_bone.lock_ik_z:
            axis = 'y'
        elif not pose_bone.lock_ik_z and pose_bone.lock_ik_x and pose_bone.lock_ik_y:
            axis = 'z'
        else:
            axis = None
            rospy.logwarn('Warning: 1 bone axis should be unlocked and 2 locked. {0} lock_ik_x: {1}, lock_ik_y: {2}, lock_ik_z: {3}'.format(pose_bone.name, pose_bone.lock_ik_x, pose_bone.lock_ik_y, pose_bone.lock_ik_z))

        return axis

    @staticmethod
    def get_bone_world_location(pose_bone):
        return (pose_bone.id_data.matrix_world * pose_bone.matrix).to_translation()

    # Assumes the origin of the armature is set to the origin of the world (need to take into account armature position)
    @staticmethod
    def set_bone_world_location(pose_bone, global_vector):
        bone = pose_bone.bone

        loc = Matrix.Translation(global_vector)
        rot = bone.matrix.to_euler().to_matrix().to_4x4()
        scale = Matrix()

        mat = loc * rot * scale
        pose_bone.matrix = mat

    @staticmethod
    def get_bone_rotation(pose_bone):
        axis = BlenderUtils.get_bone_axis(pose_bone)

        pose_mat = pose_bone.matrix.to_3x3()
        pose_mat.normalize()

        if pose_bone.parent:
            rest_mat = pose_bone.parent.matrix.to_3x3() * pose_bone.bone.matrix
            rest_mat.normalize()
        else:
            rest_mat = pose_bone.bone.matrix_local.to_3x3()

        rest_mat.transpose()
        joint_mat = rest_mat * pose_mat
        joints = joint_mat.to_euler()
        return getattr(joints, axis)

    @staticmethod
    def set_bone_rotation(pose_bone, rotation):
        pose_bone.rotation_mode = 'XYZ'
        xyz_rotation = pose_bone.rotation_euler
        axis = BlenderUtils.get_bone_axis(pose_bone)

        if axis == 'x':
            xyz_rotation[0] = rotation
        elif axis == 'y':
            xyz_rotation[1] = rotation
        elif axis == 'z':
            xyz_rotation[2] = rotation

        pose_bone.rotation_euler = xyz_rotation

    @staticmethod
    def to_blender_vector(ros_point):
        return Vector([ros_point.x, ros_point.y, ros_point.z])

    @staticmethod
    def get_joint_names(action, armature):
        joint_names = []

        for group in action.groups:
            name = group.name
            pose_bone = BlenderUtils.get_pose_bone(armature, name)

            if BlenderUtils.get_bone_axis(pose_bone) is not None:
                joint_names.append(name)

        return joint_names

    @staticmethod
    def is_keyframe(action, frame_num):
        for fcu in action.fcurves:
            for p in fcu.keyframe_points:
                if frame_num == p.co.x:
                    return True

        return False