#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file move_state.py
#   \brief This file contains the declaration of the class describing the Move state.
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
#   This file contains the declaration of the class Move and the some global variables and functions needed to describe the
#   state of the robot moving randomly in the environment.
#
import math
import random
import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from robot_simulation_state_machines.reach_goal import reachPosition
from robot_simulation_state_machines.reach_goal import planning_client
import robot_simulation_state_machines.image_processing as imp
import robot_simulation_state_machines.robot_position as rp


##
#   \brief Define the width of the house.
width = 0

##
#   \brief Define the height of the house.
height = 0

##
#   \brief fatigue_threshold defines the maximum level of fatigue the robot can andle before going to sleep.
fatigue_threshold = 0

##
#   \brief target_checking_rate defines the rate at which the reachability of the target is checked.
target_checking_rate = 40


##
#    \brief isTired check the level of fatigue to establish if the robot is "tired"
#    \param level [integer] is the current level of fatigue of the robot.
#    \return a boolen. True if the robot is "tired" false elsewhere.
#
#    This function compares the level of fatigue with and hard threshold which is a parameter
#    eccessible for the user.
#
def isTired(fatigue_level):
    if fatigue_level >= fatigue_threshold:
        return True
    else :
        return False


##
#   \brief compEuclidDist computes the Euclidean distance from two 2D position
#   \param target is the target 2D position
#   \param current is the current robot 2D position
#   \return a double corresponding to the euclidean distance.
#
def compEuclidDist(target, current):
    delta_x = target.position.x - current.position.x
    delta_y = target.position.y - current.position.y
    dist = math.sqrt( delta_x*delta_x + delta_y*delta_y )
    return dist


##
#   \class Move
#   \brief This class defines the state of the state machine corresponding to the robot randomly moving in the environment.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Move state. In this state the robot moves around randomly calling
#   the dedicated action service. As part of the smach class, this class has the member function execute()
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
    #   This member function initializes the state machine state. It follows the conventions for smach. Moreover, some members
    #   are initialized. Specifically, the subscriber to the person willing messages is defined, as well as some other internal
    #   parameters.
    #
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['tired','tracking', 'play'],
                                 input_keys=['move_fatigue_counter_in'],
                                 output_keys=['move_fatigue_counter_out','move_room_to_find'])
            self.target = Pose()
            self.targetSeemsReacheable = True
            self.prev_dist = 0.0
            self.time_to_play = False
            #   Declare the subscriber to the person willing
            self.sub = rospy.Subscriber("PersonCommand",String, self.commandReceived,  queue_size=1)

    ##
    #   \brief commandReceived is the Move member function callback for the Subscriber to the person messages
    #   \param msg is the message containing the person willing.
    #
    def commandReceived(self, msg):
        print("Received play request")
        self.time_to_play = True

    ##
    #   \brief newTarget generates a new random position and command the robot to reach it
    #   \param target specifies, if needed, a specific target to reach [default = None]
    #
    #   This function generates a new target. It uses the passed (x,y) position if given. Otherwise, it
    #   generates it randomly in within the width and height specified.
    #   It computes the current distance from the target and then makes use of the non blocking version of reachPosition()
    #   to make the robot reach the newly generated target.
    #
    def newTarget(self, target=None):
        #   Declare a geometry_msgs/Pose for the random position
        random_ = Pose()
        #   Define the random components (x, y) of this random position
        if (target == None):
            random_.position.x = random.randint(-width/2, width/2)
            random_.position.y = random.randint(-height/2, height/2)
        else:
            print("Using user-defined coordinates")
            random_.position.x = target[0]
            random_.position.y = target[1]
        #   Define the target as reacheable by default
        self.targetSeemsReacheable = True
        #   Compute the current distance
        self.target = random_
        self.prev_dist = compEuclidDist(random_, rp.robot_pose)
        #   Call the service to reach this position
        reachPosition(random_, verbose=True)



    ##
    #   \brief checkReachability control if the robot has moved forward to the target or not.
    #
    #   This function is a timer callback. It is executed periodically during the execution of this state.
    #   The procedure is to check the previously computed euclidean distance with the current one. If it has
    #   decreased, it means that the robot is moving forward the target. Otherwise, the robot is moving away from it.
    #   This last situation usually occours when the robot is following an alternative path trying to reach an unreachable target.
    #   Thus this member function changes the target using the memeber function newTarget().
    #
    def checkReachability(self, event):
        curr_dist = compEuclidDist(self.target, rp.robot_pose)
        if curr_dist >= self.prev_dist :
            print("Target seems impossible to reach. Computing a new target")
            self.newTarget()
        else:
            print("Current distance:", "%.3f"%curr_dist, "is smaller than the previus one:", "%.3f"%self.prev_dist)
            self.prev_dist = curr_dist

    ##
    #   \brief execute is main member function of the class, containing the intended behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This member function is responsible of simulating the Move behavior for the robot.
    #   In the initialization it instantiates a new target for the robot, calling the member
    #   function newTarget(). It then initializes a timer with the callback checkReachability(). This
    #   timer will then periodically call its callback and so mark the target as reacheable (or not)
    #   depending on the situation.
    #   After this initialization, it enters in a loop which can be termined by a sequence of instance checking.
    #   First, it checks whether a command from the person has been received, in which case it changes
    #   the state with 'play' which triggers the Play behavior.
    #   If a ball has been detected, then the robot is stopped and the function returns 'tracking' to move to
    #   the TrackBall state.
    #   In the case non of the previously occurred, the function gets the state of the move_base service in order to
    #   establish if the target has been reached. Once the target position has been reached, the fatigue counter is increased
    #   and it is compared with the threshold using isTired(). If the robot is tired the returning statement 'tired' brings it
    #   into the Rest behavior. Otherwise, a new target position is generated using newTarget().
    #   The timer for the reachability checking is shutdown each time this memeber function goes out of scope,
    #   in order to avoid having multiple shadow timers running in the background.
    #
    def execute(self, userdata):
        global ball_detected
        global planning_client
        global target_checking_rate
        #   Declare an empty room
        userdata.move_room_to_find = ''
        #   Declare a new target
        self.newTarget()
        #   Initialize the timer to periodically check for the robot progress
        timer = rospy.Timer(rospy.Duration(target_checking_rate), self.checkReachability)
        #   Main loop
        while not rospy.is_shutdown():
            #   First check if a command is received
            if self.time_to_play:
                timer.shutdown()
                self.time_to_play = False
                return 'play'
            #   Check if a ball has been detected
            if imp.ball_detected :
                print("Ball detected stopping the robot!")
                #   Stop the robot right there
                planning_client.cancel_all_goals()
                #   Return 'plying' to change the state
                timer.shutdown()
                return 'tracking'
            #   If none of the previous was true, then continue with the Move behavior
            #   Check if the target has been reached
            state = planning_client.get_state()
            if state == 3:
                print("Reached the position!")
                #   Increment the level of the robot fatigue
                userdata.move_fatigue_counter_out = userdata.move_fatigue_counter_in + 1
                #   Print a log to show the level of fatigue
                print('Level of fatigue : ', userdata.move_fatigue_counter_in)
                #   Check is the robot is tired
                if isTired(userdata.move_fatigue_counter_in) :
                    #   Print a log to inform about the fact that the robot is tired
                    print('Robot is tired of moving...')
                    #   Return 'tired' to change the state
                    timer.shutdown()
                    return 'tired'
                else:
                    print("Continuing to move")
                    self.newTarget()
