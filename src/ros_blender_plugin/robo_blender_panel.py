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

bl_info = {
    "name": "Robo Blender",
    "author": "Jamie Diprose (jdddog)",
    "version": (0, 0, 1),
    "blender": (2, 7, 1),
    "location": "View3D > Properties > Robo Blender",
    "description": "",
    "warning": "blah blah blah",
    "wiki_url": "http://opencog.org",
    "category": "3D View",
}

"""
Robo Blender

Usage:


"""

import bpy
from bpy.props import *
from bpy.app.handlers import persistent
from mathutils import Vector, Matrix
import rospy
from sensor_msgs.msg import JointState


class ROSNode():
    def __init__(self):
        self.sub = rospy.Subscriber('joint_states', JointState, self.update_joint_states)

    def update_joint_states(self, msg):
        if isinstance(bpy.data, bpy.types.BlendData):
            scene = bpy.data.scenes['Scene']
            active_object = scene.objects.active

            if active_object is not None:
                if active_object.type == 'ARMATURE' and active_object.mode == 'POSE':
                    armature = active_object

                    if scene.robo_blender_pose_source == 'robot':
                        self.enable_constraints(False)

                        for i, joint_name in enumerate(msg.name):
                            position = msg.position[i]
                            bone = self.get_bone(armature, joint_name)
                            self.set_bone_position(bone, position)
                    else:
                        self.enable_constraints(True)

<<<<<<< HEAD:src/ros_blender_plugin/robo_blender_panel.py


                # Check if need to change state of motors
                gui_enable_motors = scene.robo_blender_motors == 'enable'
=======
    def get_bone(self, armature, joint_name):
        return armature.pose.bones[joint_name]
>>>>>>> c441a42e2da0bb3832ffdf97d5a051e2d0f567a7:scripts/robo_blender_panel.py

    def set_bone_position(self, bone, position):
        bone.rotation_mode = 'XYZ'
        rotation = bone.rotation_euler

        if not bone.lock_ik_x and bone.lock_ik_y and bone.lock_ik_z:
            rotation[0] = position
        elif not bone.lock_ik_y and bone.lock_ik_x and bone.lock_ik_z:
            rotation[1] = position
        elif not bone.lock_ik_z and bone.lock_ik_x and bone.lock_ik_y:
            rotation[2] = position
        else:
            rospy.logwarn('Warning: 1 bone axis should be unlocked and 2 locked. {0} lock_ik_x: {1}, lock_ik_y: {2}, lock_ik_z: {3}'.format(bone.name, bone.lock_ik_x, bone.lock_ik_y, bone.lock_ik_z))

        bone.rotation_euler = rotation

    def enable_constraints(self, armature, enable):
        for bone in armature.pose.bones:
            for constraint in bone.constraints:
                if constraint.mute is not enable:
                    constraint.mute = enable


class RoboBlenderPanel(bpy.types.Panel):

    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = "Robo Blender"
    bl_options = {'DEFAULT_CLOSED'}

    MOTORS = [
    ("disable", "Disable", "Disable leg motors"),
    ("enable", "Enable", "Enable leg motors")]

    POSE_SOURCE = [
    ("blender", "Blender", "Use blender to generate armature pose"),
    ("robot", "Robot", "Use robot to generate armature pose")]

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        scene = context.scene

        # Choose pose source
        row = layout.row(align=True)
        row.label(text="Source:")
        row.prop(scene, "robo_blender_pose_source", expand=True)

    @classmethod
    def poll(cls, context):
        return True


def register():
    scene = bpy.types.Scene
    bpy.utils.register_module(__name__)

    scene.robo_blender_pose_source = bpy.props.EnumProperty(
        name="pose_source",
        description="Choose the source of the armatures pose",
        items=RoboBlenderPanel.POSE_SOURCE,
        default='blender')

    global ros_node
    ros_node = ROSNode()

def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.robo_blender_pose_source
    del ros_node

if __name__ == "__main__":
    register()

