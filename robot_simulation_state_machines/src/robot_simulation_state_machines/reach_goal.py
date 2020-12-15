#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import roslib
import rospy

# Ros Messages
from geometry_msgs.msg import Pose



# Brings in the SimpleActionClient
import actionlib
import robot_simulation_messages.msg




##
#   \brief Creates the SimpleActionClient, passing the type of the action (PlanningAction) to the constructor.
#
#   It is defined as global in order to be used with a global function
planning_client = actionlib.SimpleActionClient('reaching_goal', robot_simulation_messages.msg.PlanningAction)

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
    goal = robot_simulation_messages.msg.PlanningGoal(pose)
    #   Sends the goal to the action server.
    planning_client.send_goal(goal)
    #   Waits for the server to finish performing the action.
    if wait:
        planning_client.wait_for_result()
