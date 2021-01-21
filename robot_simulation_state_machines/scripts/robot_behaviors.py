#!/usr/bin/env python

## @package robot_simulation_state_machines
#   \file robot_behaviors.py
#   \brief This file contains the state machine for the states defining the robot behaviors.
#   \author Andrea Gotelli
#   \version 0.2
#   \date 22/10/2020
#
#   \param [in] world_width
#   \param [in] world_height
#   \param [in] sleep_x_coord
#   \param [in] sleep_y_coord
#   \param [in] fatigue_threshold
#   \param [in] maximum_dead_time
#   \param [in] max_speed
#
#   \details
#
#   Subscribes to: <BR>
#        camera1/image_raw/compressed
#        joint_neck_position_controller/state
#        odom
#
#   Publishes to: <BR>
#        joint_neck_position_controller/command
#        cmd_vel
#
#   Service : <BR>
#        [None]
#
#   Description :
#
#   This file uses smach libraries to generate two state machines, which are used to control the
#   behaviors of the robot. Specifically, the state machines defines the transition from a state to
#   another using the interface provided by smach. There is a top level state machine defining two
#   of the three main robot behaviors. The last behavior is defined by the states of a sub state
#   machine.
#   The states are defined in the respective classes, and the transistions are automatically performed
#   by the state machine from the smach libraries.
#
# This Python file uses the following encoding: utf-8



# /move_base_simple/goal


import roslib
import rospy
import smach
import smach_ros
import time
import random
import math

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf import transformations
from control_msgs.msg import JointControllerState

# Brings in the SimpleActionClient
import actionlib
import robot_simulation_messages.msg

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
import imutils

# OpenCV
import cv2

from robot_simulation_state_machines.reach_goal import reachPosition
from robot_simulation_state_machines.reach_goal import planning_client
from robot_simulation_messages.srv import PersonCommand
from robot_simulation_messages.srv import ShowAvailableRooms, ShowAvailableRoomsResponse
from explore_lite.msg import ExploreAction, ExploreGoal



#from robot_simulation_state_machines.image_processing import imageReceived
import robot_simulation_state_machines.image_processing as imp
from robot_simulation_state_machines.image_processing import imageReceived, findBallIn, registerRoom, sarCallback

import robot_simulation_state_machines.move_state as ms
from robot_simulation_state_machines.move_state import Move

from robot_simulation_state_machines.track_ball_state import TrackBall, laserReadingCallback
import robot_simulation_state_machines.track_ball_state as tbs

from robot_simulation_state_machines.rest_state import Rest, sleep_station

from robot_simulation_state_machines.find_state import Find

from robot_simulation_state_machines.interact_state import Interact

from robot_simulation_state_machines.robot_position import odometryReceived

import robot_simulation_state_machines.robot_position as rp




"""
    Problems:
        -> Ci sta un po a assumere positione raggiunta //
        -> Non posso usare odometry//
        -> A volte si pianta contro i muri//


    Questions:
        -> How to move classes in files... Problems with global and initializations...//
        -> Is it ok to have two states referring to the same class?//
        -> How to increase the level of fatigue<? each time?//
        -> Still crashing into walls sometimes//
        -> If detects a ball and then another one?
        -> How to pass references in phyton?
        -> The usage of explore lite//

"""

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
    global sleep_station
    global fatigue_threshold
    global last_detection
    global maximum_dead_time
    global neck_controller
    global neck_controller_rate
    global robot_controller
    global max_speed
    global time_before_change_target

    #   Initialization of the ros node
    rospy.init_node('robot_behavior_state_machine')

    #   Sleep for waiting the end of all the Initialization logs
    rospy.sleep(0.5)
    print("Starting the state machines")
    #   Initialize the seed
    random.seed()

    #   Dummy Initialization
    imp.last_detection = rospy.Time.now().to_sec()

    #   Subscribed to the recorded image
    image_subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                            CompressedImage, imageReceived,  queue_size=1)

    #   Definition of the pubisher for the robot velocity
    tbs.robot_controller = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #   Definition of the subscriber to the laser scan
    sub_laser = rospy.Subscriber('/scan', LaserScan, laserReadingCallback, queue_size=1)


    #   Definition of the subscriber for the robot Odometry
    sub_odom = rospy.Subscriber('odom', Odometry, odometryReceived)


    #############################################################
    sar_service = rospy.Service('sar_service', ShowAvailableRooms, sarCallback)
    ############################################################à

    #   Retrieve the parameter about the world dimensions
    ms.width = rospy.get_param('/world_width', 20)
    ms.height = rospy.get_param('/world_height', 20)

    #   Retrieve parameters about the sleeping position
    sleep_station.position.x = rospy.get_param('/sleep_x_coord', 0)
    sleep_station.position.y = rospy.get_param('/sleep_y_coord', 0)

    #   Retrieve the parameters of the fatigue threshold and max dead time
    ms.fatigue_threshold = rospy.get_param('/fatigue_threshold', 5)
    tbs.maximum_dead_time = rospy.get_param('/maximum_dead_time', 5)

    #   Robot maximum linear velocity
    imp.max_speed = rospy.get_param('/max_speed', 0.5)

    print("fatigue_threshold" , ms.fatigue_threshold)
    print("maximum_dead_time" , tbs.maximum_dead_time)

    ms.time_before_change_target = rospy.get_param('/time_before_change_target', 40.0)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['behavior_interface'])
    sm.userdata.fatigue_level = 0

    # Open the container
    with sm:
        # Add states to the container

        #   Create the sub state machine for the normal behavior
        sub_sm = smach.StateMachine(outcomes=['sleepy_robot', 'time_to_play'],
                                    output_keys=['sub_sm_fatigue_level'],
                                    input_keys=['sub_sm_fatigue_level'])

        with sub_sm:

            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'tired':'sleepy_robot',
                                                'tracking':'TRACK_BALL',
                                                'play':'time_to_play'},
                                   remapping={'move_fatigue_counter_in':'sub_sm_fatigue_level',
                                              'move_fatigue_counter_out':'sub_sm_fatigue_level',
                                              'move_room_to_find':'room_to_find'})

            smach.StateMachine.add('TRACK_BALL', TrackBall(),
                                   transitions={'tired':'sleepy_robot',
                                                'registered':'MOVE',
                                                'room_founded':'MOVE',
                                                'ball_lost':'MOVE'},
                                   remapping={'track_ball_fatigue_counter_in':'sub_sm_fatigue_level',
                                              'track_ball_fatigue_counter_out':'sub_sm_fatigue_level',
                                              'track_room_to_find':'room_to_find'})



        smach.StateMachine.add('NORMAL', sub_sm,
                               transitions={'sleepy_robot':'REST',
                                            'time_to_play':'PLAY'},
                               remapping={'sub_sm_fatigue_level':'fatigue_level',
                                          'sub_sm_fatigue_level':'fatigue_level'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'NORMAL'},
                               remapping={'rest_fatigue_counter_in':'fatigue_level',
                                          'rest_fatigue_counter_out':'fatigue_level'})

        play_sub_sm = smach.StateMachine(outcomes=['sleepy_robot', 'restart_moving'],
                              output_keys=['play_sub_sm_fatigue_level'],
                              input_keys=['play_sub_sm_fatigue_level'])

        with play_sub_sm :

            smach.StateMachine.add('INTERACT', Interact(),
                                   transitions={'find':'FIND',
                                                'stop_play':'restart_moving',
                                                'tired':'sleepy_robot'},
                                   remapping={'interact_fatigue_counter_in':'play_sub_sm_fatigue_level',
                                              'interact_fatigue_counter_out':'play_sub_sm_fatigue_level',
                                              'interact_ball_to_find':'room_to_find'})

            smach.StateMachine.add('FIND', Find(),
                                   transitions={'track':'TRACK_BALL'})

            smach.StateMachine.add('TRACK_BALL', TrackBall(),
                                   transitions={'tired':'sleepy_robot',
                                                'registered':'FIND',
                                                'room_founded':'INTERACT',
                                                'ball_lost':'FIND'},
                                   remapping={'track_ball_fatigue_counter_in':'play_sub_sm_fatigue_level',
                                              'track_ball_fatigue_counter_out':'play_sub_sm_fatigue_level',
                                              'track_room_to_find':'room_to_find'})

        smach.StateMachine.add('PLAY', play_sub_sm,
                              transitions={'sleepy_robot':'REST',
                                           'restart_moving':'NORMAL'},
                              remapping={'play_sub_sm_fatigue_level':'fatigue_level',
                                         'play_sub_sm_fatigue_level':'fatigue_level'})




    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('robot_behavior_state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
