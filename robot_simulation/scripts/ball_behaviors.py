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



##
#   \brief Define the width of the discretized world. It is a parameter defined by the user.
width = 0

##
#   \brief Define the height of the discretized world. It is a parameter defined by the user.
height = 0

number_of_movements = 0


# Creates the SimpleActionClient, passing the type of the action
# (PlanningAction) to the constructor.
planning_client = actionlib.SimpleActionClient('reaching_goal', robot_simulation_messages.msg.PlanningAction)
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

def waitForRandTime(min=10, max=20):
    print("min : ", min, " max : ", max)
    rospy.sleep( random.randint(min, max) )


##
#   \class Move
#   \brief This class defines the state of the state machine corresponding to the robot randomly moving
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Move state. In this state the robot moves around randomly calling
#   the service /MoveToPosition. As part of the smach class, this class has the member function execute()
#   providing the intended behavior. For more details about the content of this class, see the member functions
#   documentation.
#
class Move(smach.State):
    ##
    #   \brief __init__ initializes the class and its members
    #   \param outcomes list of the possible outcomes from this state of the state machine.
    #   \param input_keys list of the possible input for this state (user data shared among states).
    #   \param output_keys list of the possible outputs for this state (user data shared among states).
    #
    #   This member function initializes the state machine state. It follows the conventions for smach.
    #   Moreover, it initializes the subscriber necessary to receive a command from the person.
    #
    def __init__(self):
            smach.State.__init__(self, outcomes=['hide'])


    ##
    #   \brief execute is main member function of the class, containing the intended behavior
    #   \return a string consisting of the state outcome
    #
    #
    #
    def execute(self, userdata):
        global number_of_movements
        print("Number of movements : ", number_of_movements)
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
#   \class Rest
#   \brief This class defines the state of the state machine corresponding to the robot sleeping.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Rest state. Where the pet like robot rests for recover energies.
#   As part of the smach class, this class has the member function execute() providing the intended
#   behavior. For more details about the content of this class, see the member function documentation.
#
class Rest(smach.State):
    ##
    #   \brief The __init__ constructor initializes the state outcome and input output keys
    def __init__(self):
            smach.State.__init__(self, outcomes=['move'])
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This simple member function moves the robot in the sleeping position and waits for some time
    #   in order to simulate the pet like robot sleeping. It also resets the fatigue level to zero
    #   to let the robot to move and performs his behaviors.
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
#   \brief __main__ intializes the ros node and the smach state machine
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

        smach.StateMachine.add('HIDE', Rest(),
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

