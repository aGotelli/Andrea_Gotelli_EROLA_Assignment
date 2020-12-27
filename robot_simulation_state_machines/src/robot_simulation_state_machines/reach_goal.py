#!/usr/bin/env python



## @package robot_simulation_state_machines
#   \file reach_goal.py
#   \brief This file contains the a function allowing to simple call to the action service.
#   \author Andrea Gotelli
#   \version 0.2
#   \date 22/10/2020
#
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
#        /reaching_goal as client, it asks for reach a specific location.
#
#   Description :
#
#   This file contains an action service client which calls and eventually waits for the action service
#   result. This choice was made in order to make this function available in more moduls.
#
# This Python file uses the following encoding: utf-8

import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped



# Brings in the SimpleActionClient
import actionlib
import robot_simulation_messages.msg
#import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




##
#   \brief Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor.
#
#   It is defined as global in order to be used with a global function
#planning_client = actionlib.SimpleActionClient('/move_base', move_base_msgs.msg.MoveBaseAction)
planning_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

##
#   \brief reachPosition Is a global function which calls the appropriate action service
#   \param pose Is the pose to reach
#   \param wait default False, if true makes the function blocking by calling the action server
#                memeber function wait_for_result()
#   \param verbose default False, if true it make the function to print some logs
#                   about the position to reach
#
#   This function serves as a compact way to call the action service. It waits for the service
#   to exist, it gives the position to be reached and it may waits for the results, depending
#   on the boolean state of the relative parameter.
def reachPosition(pose, wait=False, verbose=False):
    global planning_client
    if verbose :
        #   Print a log of the given postion
        print("Reaching position: ", pose.position.x , ", ", pose.position.y)
    #   Waits until the action server has started up and started
    #   listening for goals.
    planning_client.wait_for_server()
    #   Creates a goal to send to the action server.
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose.position.x
    goal.target_pose.pose.position.y = pose.position.y
    goal.target_pose.pose.orientation.w = pose.position.y

    planning_client.send_goal(goal)

    if wait:
        planning_client.wait_for_result()
