#!/usr/bin/env python
# Copyright (c) 2014, Mandeep Bhatia, Jamie Diprose
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
    "author": "Mandeep Bhatia, Jamie Diprose (jdddog)",
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
from threading import Thread
#import rospy #TODO: enable this code
#from sensor_msgs.msg import JointState #TODO: enable this code


class BackgroundThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.enable_motors = False #TODO: Check if motor state with robot, if not same as self.motors_enabled, enable / disable them
        self.active = False
        #self.joint_states = JointState() #TODO: enable this code
        #self.rate = rospy.Rate(10) #TODO: enable this code
        #rospy.Subscriber('joint_states', JointState, self.update_joint_states) #TODO: enable this code

    def update_joint_states(self, msg):
        self.joint_states = msg

    def run(self):
        self.active = True

        while self.active:
            if isinstance(bpy.data, bpy.types.BlendData):
                scene = bpy.data.scenes['Scene']
                active_object = scene.objects.active



                # Check if need to change state of motors
                gui_enable_motors = scene.robo_blender_motors == 'enable'

                if not (self.enable_motors == gui_enable_motors):
                    self.enable_motors = gui_enable_motors
                    print("motor_state: " + scene.robo_blender_motors)
                    #TODO: Set state of motors to gui_enable_motors value

                # If armature selected, and pose mode on
                if active_object is not None:
                    if active_object.type == 'ARMATURE' and active_object.mode == 'POSE':
                        gui_pose_source = scene.robo_blender_pose_source

                        if gui_pose_source == 'robot':
                            # TODO: Disable ik chain on legs
                            # TODO: Loop through joint state message, for each joint set the position of the bone with the same name
                            print("Setting armature pose from robot's motors")
                        else:
                            print("Setting armature pose from blender")
                            # TODO: Make sure IK chains on legs are enabled

    def stop(self):
        self.active = False


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

        # Enable disable motors
        row = layout.row(align=True)
        row.label(text="Motors:")
        row.prop(scene, "robo_blender_motors", expand=True)

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

    scene.robo_blender_motors = bpy.props.EnumProperty(
        name="motors",
        description="Enable or disable robot motors",
        items=RoboBlenderPanel.MOTORS,
        default='disable')

    scene.robo_blender_pose_source = bpy.props.EnumProperty(
        name="pose_source",
        description="Choose the source of the armatures pose",
        items=RoboBlenderPanel.POSE_SOURCE,
        default='blender')

    global background_thread
    background_thread = BackgroundThread()
    background_thread.start()

def unregister():
    bpy.utils.unregister_module(__name__)
    del bpy.types.Scene.robo_blender_motors
    del bpy.types.Scene.robo_blender_pose_source
    background_thread.stop()


if __name__ == "__main__":
    register()

