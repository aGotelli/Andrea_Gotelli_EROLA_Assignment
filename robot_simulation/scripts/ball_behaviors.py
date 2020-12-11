## @package state_machine
#   \file state_machine.py
#   \brief This file contains the state machine for the high level control of this application.
#   \author Andrea Gotelli
#   \version 0.2
#   \date 22/10/2020
#
#   \param [in] world_width the width of the discretized world
#   \param [in] world_height the height of the discretized world
#   \param [in] sleep_x_coord is the x coordinate of the position where the robot sleeps
#   \param [in] sleep_y_coord is the y coordinate of the position where the robot sleeps
#
#   \details
#
#   Subscribes to: <BR>
#       ° /PlayWithRobot topic where the person publishes the given play command
#
#   Publishes to: <BR>
#       ° [None]
#
#   Service : <BR>
#       ° /Gesture as client, it waits for the gesture to come.
#
#       ° /MoveToPosition as cliet, ask to simulate the motion to the given position
#
#   Description :
#
#   This file uses smach libraries to generate a state machine, which is used to control the
#   behaviors of the robot. Specifically, the state machine defines the transition from a state to
#   another using the interface provided by smach.
#
#   This file could be considered the head of the simulation an its more complex and important part.
#
#   The states are defined in the respective classes, and the transistions are automatically performed
#   by the state machine from the smach libraries.
#

#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import roslib
import rospy
import smach
import smach_ros
import time
import random
import math

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float64
from robot_simulation_messages.srv import MoveTo
from robot_simulation_messages.msg import PersonCalling
from robot_simulation_messages.srv import GiveGesture

# Brings in the SimpleActionClient
import actionlib
import robot_simulation_messages.msg


# numpy and scipy
import numpy as np

#from reach_goal import reachPosition



##
#   \brief Define the width of the discretized world.
#
#   It is a parameter that the user may change with the use of the correct ROS parameter.
width = 0

##
#   \brief Define the height of the discretized world.
#
#   It is a parameter that the user may change with the use of the correct ROS parameter.
height = 0

##
#   \brief Define how many movements the ball will perform before hiding.
#
#   It is a parameter that the user may change with the use of the correct ROS parameter.
number_of_movements = 0


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
        print("Ball is reaching position: ", pose.position.x , ", ", pose.position.y)
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


##
#   \brief waitForRandTime Global blocking function
#   \param min Default 10, is the minimum time, in seconds
#   \param max Default 20, is the maximum time, in seconds
#
#   This function only calls the rospy function sleep(t) which
#   stops the code for a time t. this time is computed randomly
#   in the interval given by the min and max parameters.
#   This interval is [10 20] by default.
def waitForRandTime(min=10, max=20):
    rospy.sleep( random.randint(min, max) )


##
#   \class Move
#   \brief This class defines the state of the state machine corresponding to the ball randomly moving
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Move state. In this state the ball moves around randomly.
#   As part of the smach class, this class has the member function execute()
#   providing the intended behavior. For more details about the content of this class, see the member functions
#   documentation.
#
class Move(smach.State):
    ##
    #   \brief __init__ initializes the class and its members
    #
    #   This member function initializes the state machine state. It follows the conventions for smach.
    #   It calls the the initializer of the smach State class passing the unique outcome from this state.
    #
    def __init__(self):
            smach.State.__init__(self, outcomes=['hide'])


    ##
    #   \brief execute is main member function of the class, containing the intended behavior
    #   \param userdata Unused parameter in order to pass userdata to the function.
    #   \return a string consisting of the state outcome
    #
    #   This member function enters in two subsequent loops. The first loop ensures that the
    #   ROS enviroment is still available, i.e. that the roscore is still operating, and
    #   it enters in a second loop which iterates for a fixed number of time the moving
    #   sequence. The number of time is given by the global variable number_of_movements.
    #
    #   The moving sequence is simply obtained by the creation of a random position within the
    #   world, and then the position is fed into the function reachPosition.
    #
    #   After the last motion, the function returns a string in order to change the state into "hide"
    #
    def execute(self, userdata):
        global number_of_movements
        while not rospy.is_shutdown():
            for index in range(number_of_movements):
                #   Declare a geometry_msgs/Pose for the random position
                random_ = Pose()
                #   Define the random components (x, y) of this random position
                random_.position.x = random.randint(-width/2, width/2)
                random_.position.y = random.randint(-height/2, height/2)
                random_.position.z = 1
                #   Call the service to reach this position
                reachPosition(random_, wait=True)
            return 'hide'







##
#   \class Hide
#   \brief This class defines the state of the state machine corresponding to the robot sleeping.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Hide state. In this state, the ball is hidden from the robot.
#   As part of the smach class, this class has the member function execute() providing the intended
#   behavior. For more details about the content of this class, see the member function documentation.
#
class Hide(smach.State):

    ##
    #   \brief __init__ initializes the class and its members
    #
    #   This member function initializes the state machine state. It follows the conventions for smach.
    #   It calls the the initializer of the smach State class passing the unique outcome from this state.
    #
    def __init__(self):
            smach.State.__init__(self, outcomes=['move'])
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata Unused parameter in order to pass userdata to the function.
    #   \return a string consisting of the state outcome
    #
    #   This simple member function moves the ball two meters below the floor in order to make it
    #   disappear for the robot. It simply creates the position two meters below the origin of the
    #   world frame and it fed it as a parameter of the function reachPosition. In this case the
    #   parameter wait is setted to true in order to make the function blocking. Moreover, it
    #   waits a random amount of time calling the function waitForRandTime with default parameters.
    #
    def execute(self, userdata):
        #   Declare a geometry_msgs/Pose for the random position
        random_ = Pose()
        #   Define the random components (x, y) of this random position
        random_.position.x = 0
        random_.position.y = 0
        random_.position.z = -2
        #   Call the service to reach this position
        reachPosition(random_, wait=True)
        #   Wait some time before showing ball again
        waitForRandTime()
        return 'move'



##
#   \brief main intializes the ros node and the smach state machine
#
#   This functions does nothing more than initializing the node and the state machine.
#   It also retrieves the parameters from the ros parameters server and the user data
#   exchanged among the state machine states.
#
def main():
    global width
    global height
    global number_of_movements
    #   Initialization of the ros node
    rospy.init_node('robot_behavior_state_machine')

    #   Sleep for waiting the end of all the Initialization logs
    waitForRandTime(15, 25)
    print("Starting to move the ball")
    random.seed()

    #   Retrieve the parameter about the world dimensions
    width = rospy.get_param('/world_width', 20)
    height = rospy.get_param('/world_height', 20)
    number_of_movements = rospy.get_param('/number_of_movements', 5)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['ball_behavior_interface'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'hide':'HIDE'})

        smach.StateMachine.add('HIDE', Hide(),
                               transitions={'move':'MOVE'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('ball_behavior_state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()

