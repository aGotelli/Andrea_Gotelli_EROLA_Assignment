#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file robot_position.py
#   \brief This file contains the class declaration for the robot position obtained from odometry
#   \author Andrea Gotelli
#   \version 0.2
#   \date 27/01/2021
#
#   \details
#
#   Subscribes to: <BR>
#        [None]
#
#   Publishes to: <BR>
#        [None]
#
#   Service : <BR>
#        [None]
#
#   Description :
#
#   This file contains only a Subscriber callback and a global variable. The global variable correspons to the robot pose
#   which is needed in multiple subfiles of this application. Creating it in a separated file allows to make is avaiable to all.
#

from geometry_msgs.msg import Pose

##
#   \brief robot_pose is the robot pose in the world, obtained from odometry
robot_pose = Pose()

##
#    \brief odometryReceived stores the value of the current robot pose
#    \param msg is the robot current odometry
#
#    This function simply copy the robot pose from the odometry to a global variable.
#
def odometryReceived(msg):
    global robot_pose
    robot_pose = msg.pose.pose
