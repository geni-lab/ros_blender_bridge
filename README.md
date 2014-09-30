Welcome to ROS Blender Bridge!
===
ROS Blender Bridge is a ROS driver for Blender. It provides access to Blender armatures in ROS, allows you to run Blender produced animations on ROS based robots and enables you to use Blender's inverse kinematics system to target robot body parts at objects in the robots environment (e.g. to make a robot gaze at something).

# Learning ROS Blender Bridge
We've created a set of a set of tutorials to help you get started with ROS Blender Bridge. It is assumed that you have already installed ROS and have a catkin workspace setup. If you have not installed ROS [click here](http://wiki.ros.org/indigo/Installation/Ubuntu) and if you do not have a catkin workspace setup [click here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

1. [Installing ROS Blender Bridge](https://github.com/geni-lab/ros_blender_bridge/wiki/Installation): This tutorial teaches you how to install ROS Blender bridge and its dependencies.
2. [Creating a Blender armature](https://github.com/geni-lab/ros_blender_bridge/wiki/Creating-an-armature-in-Blender): This tutorial teaches you how to create a Blender armature that is compatible with ROS Blender Bridge.
3. [Accessing the joint state of an armature](): This tutorial teaches you how to access the joint state of an armature from ROS.
4. [Configuring a target controller](): This tutorial teaches you how to setup a target controller so that you can make a robot gaze at a target.
5. [Creating Actions and using them in ROS](): This tutorial teaches you to create a Blender Action animation and describes how to generate a JointTrajectory message to play the animation back on a robot.
