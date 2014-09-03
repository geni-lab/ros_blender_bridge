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

import bpy
import mathutils
from mathutils import Matrix, Vector
from math import acos, degrees
from xml.dom.minidom import parseString
from xml.etree.ElementTree import Element, SubElement, Comment, tostring


class LinkRef(object):
    def __init__(self, name, link_type):
        self.name = name
        self.link_type = link_type

    def to_xml(self, xml_parent):
        link_ref = SubElement(xml_parent, self.link_type)
        link_ref.set('link', self.name)
        return link


class Origin(object):
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def to_xml(self, xml_parent):
        origin = SubElement(xml_parent, 'origin')
        origin.set('xyz', "{0} {1} {2}".format(self.x, self.y, self.z))
        origin.set('rpy', "{0} {1} {2}".format(self.roll, self.pitch, self.yaw))
        return origin


class Axis(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_xml(self, xml_parent):
        axis = SubElement(xml_parent, 'axis')
        axis.set('xyz', "{0} {1} {2}".format(self.x, self.y, self.z))
        return axis


class Limit(object):
    def __init__(self, velocity, effort, lower, upper):
        self.velocity = velocity
        self.effort = effort
        self.lower = lower
        self.upper = upper

    def to_xml(self, xml_parent):
        limit = SubElement(xml_parent, 'limit')
        limit.set('velocity', str(self.velocity))
        limit.set('effort', str(self.effort))
        limit.set('lower', str(self.lower))
        limit.set('upper', str(self.upper))
        return limit


class Joint(object):
    def __init__(self, name, type, parent_link, child_link, origin, axis, limit):
        self.name = name
        self.type = type
        self.parent_link = parent_link
        self.child_link = child_link
        self.origin = origin
        self.axis = axis
        self.limit = limit

    @staticmethod
    def to_link_name(name):
        return name.replace('_joint', '_link')

    def to_xml(self, xml_parent):
        joint = SubElement(xml_parent, 'joint')
        joint.set('name', self.name)
        joint.set('type', self.type)
        self.parent_link.to_xml(joint)
        self.child_link.to_xml(joint)
        self.origin.to_xml(joint)
        self.axis.to_xml(joint)
        self.limit.to_xml(joint)

        return joint


def get_root_bone(bones):
    for bone in bones:
        if bone.parent is None:
            return bone


def get_bone(name, bones):
    for bone in bones:
        if bone.name == name:
            return bone


def is_joint(bone):
    return bone.name.endswith('joint')


def to_ros_coord(x, y, z):
    return (z, -x, y)


def add_prefix(name):
    if name.startswith('l_'):
        return name.replace('l_', '${prefix}_')
    elif name.startswith('r_'):
        return name.replace('r_', '${prefix}_')

    return name


class Link(object):
    def __init__(self, name):
        self.name = name

    def to_xml(self, xml_parent):
        link = SubElement(xml_parent, 'name')
        link.set('name', self.name)
        return link

# joints, links
    #if is_root:
    #    global visited_joints
    #    global links
    #    links = []
    #    visited_joints = {}
        #joints = []
        #    visited_joints[parent_bone.name] = True


def has_been_visited(name, joints):
    visited = False

    for joint in joints:
        if name == joint.name:
            visited = True
            break

    return visited


def generate_links_and_joints(parent_pose_bone, links=[], joints=[]):
    try:
        if len(visited_joints) == 0:
            global visited_joints
            visited_joints = {}
    except NameError:
        global visited_joints
        visited_joints = {}

    print(len(visited_joints))

    visited_joints[parent_pose_bone.name] = True

    if is_joint(parent_pose_bone):
        parent_edit_bone = parent_pose_bone.bone

        for child_pose_bone in parent_pose_bone.children:
            child_edit_bone = child_pose_bone.bone

            if is_joint(child_pose_bone) and child_pose_bone.name not in visited_joints:

                # Parent & child
                parent_link = LinkRef(Joint.to_link_name(parent_pose_bone.name), 'parent')
                child_link = LinkRef(Joint.to_link_name(child_pose_bone.name), 'child')

                # Origin
                dX = round(parent_pose_bone.head[0] - child_pose_bone.head[0], 4)
                dY = round(parent_pose_bone.head[1] - child_pose_bone.head[1], 4)
                dZ = round(parent_pose_bone.head[2] - child_pose_bone.head[2], 4)
                point = to_ros_coord(dX, dY, dZ)

                #rot = parent_edit_bone.localOrientation.to_euler()

                #parent_pose_bone.worldOrientation

                #matrix_final = parent_edit_bone.id_data.matrix_world * parent_edit_bone.matrix

                #angles = get_bone_rotation(parent_edit_bone)

                mat = child_pose_bone.id_data.matrix_world

                #print(str((parent_pose_bone.matrix).to_euler()))

                print("angle of " + child_pose_bone.name + ": " + str((mat * child_pose_bone.matrix).to_euler()) )

                origin = Origin(point[0], point[1], point[2], 0, 0, 0)




                axis = Axis(1, 0, 0)
                limit = Limit(0, 0, 0, 0)

                # Joint
                joint = Joint(child_pose_bone.name, 'revolute', parent_link, child_link, origin, axis, limit)
                joints.append(joint)

                link = Link(Joint.to_link_name(child_pose_bone.name))
                links.append(link)

                (joints, links) = generate_links_and_joints(child_pose_bone, links, joints)

                print("{0} to {1}: ({2}, {3}, {4}, {5})".format(child_pose_bone.name, child_edit_bone.name, point[0], point[1], point[2], parent_pose_bone.vector))

    return (joints, links)

def pretty_print(xml_element):
    ugly_str = tostring(xml_element, 'utf-8')
    mini_dom_str = parseString(ugly_str)
    return mini_dom_str.toprettyxml(indent="\t")

rig = bpy.data.objects['Armature']
root_pose_bone = get_root_bone(rig.pose.bones)
print("root_bone: " + str(root_pose_bone))
print("is joint: " + str(is_joint(root_pose_bone)))


robot = Element('robot')
robot.set('xmlns:xacro', 'http://ros.org/wiki/xacro')
macro = SubElement(robot, 'xacro:macro')
macro.set('name', 'blender_generated_urdf')

(joints, links) = generate_links_and_joints(root_pose_bone)

print(len(joints))

bone = get_bone('big_bone', rig.pose.bones )
print("BONE: " + str((bone.id_data.matrix_world * bone.matrix).to_euler()))

for link in links:
    link.to_xml(macro)

for joint in joints:
    joint.to_xml(macro)