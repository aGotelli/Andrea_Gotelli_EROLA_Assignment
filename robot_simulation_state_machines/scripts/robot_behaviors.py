#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
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
#   \param [in] max_explore_time
#   \param [in] max_play_time
#   \param [in] target_checking_rate
#
#   \details
#
#   Subscribes to: <BR>
#        camera1/image_raw/compressed
#        odom
#
#   Publishes to: <BR>
#        cmd_vel
#
#   Service : <BR>
#        [None]
#
#   Description :
#
#   This file uses smach libraries to generate a nested state machine which is able to simulate the robot behaviors.
#   The states machines are defined following the rules of the smach library. The class that they uses in order to
#   instantiate every state and behavior for the robot are taken from the respective file.
#

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


import robot_simulation_state_machines.image_processing as imp
from robot_simulation_state_machines.image_processing import imageReceived, findBallIn, registerRoom, sarCallback

import robot_simulation_state_machines.move_state as ms
from robot_simulation_state_machines.move_state import Move

import robot_simulation_state_machines.track_ball_state as tbs
from robot_simulation_state_machines.track_ball_state import TrackBall, laserReadingCallback

import robot_simulation_state_machines.rest_state as rs
from robot_simulation_state_machines.rest_state import Rest

import robot_simulation_state_machines.robot_position as rp
from robot_simulation_state_machines.robot_position import odometryReceived

import robot_simulation_state_machines.explore_state as ex
from robot_simulation_state_machines.explore_state import Explore

import robot_simulation_state_machines.play_state as pl
from robot_simulation_state_machines.play_state import Play


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

    #   Definition of the service to retrive the available rooms
    sar_service = rospy.Service('sar_service', ShowAvailableRooms, sarCallback)

    #   Retrieve the parameter about the world dimensions
    ms.width = rospy.get_param('/world_width', 20)
    ms.height = rospy.get_param('/world_height', 20)

    #   Retrieve parameters about the sleeping position
    rs.sleep_station.position.x = rospy.get_param('/sleep_x_coord', 0)
    rs.sleep_station.position.y = rospy.get_param('/sleep_y_coord', 0)

    #   Retrieve the parameters of the fatigue threshold and max dead time
    ms.fatigue_threshold = rospy.get_param('/fatigue_threshold', 5)
    tbs.maximum_dead_time = rospy.get_param('/maximum_dead_time', 2)

    #   Declare the service client
    pl.person_srv_client = rospy.ServiceProxy('/person_decision', PersonCommand)

    #   Retrieve max time in Play and Explore
    pl.max_play_time = rospy.get_param('/max_play_time', 600)
    ex.max_explore_time = rospy.get_param('/max_explore_time', 300)

    #   Robot maximum linear velocity
    imp.max_speed = rospy.get_param('/max_speed', 0.5)
    imp.start_time = rospy.Time.now().to_sec()

    print("fatigue_threshold" , ms.fatigue_threshold)
    print("maximum_dead_time" , tbs.maximum_dead_time)

    ms.target_checking_rate = rospy.get_param('/target_checking_rate', 40.0)

    # Create a SMACH state machine
    robot_behaviors = smach.StateMachine(outcomes=['behavior_interface'])
    robot_behaviors.userdata.fatigue_level = 0
    robot_behaviors.userdata.start_play_time = 0

    # Open the container
    with robot_behaviors:
        # Add states to the container

        #   Create the sub state machine for the normal behavior
        normal_sub_sm = smach.StateMachine(outcomes=['sleepy_robot', 'time_to_play'],
                                    output_keys=['normal_ssm_fatigue_level'],
                                    input_keys=['normal_ssm_fatigue_level'])

        with normal_sub_sm:

            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'tired':'sleepy_robot',
                                                'tracking':'TRACK_BALL',
                                                'play':'time_to_play'},
                                   remapping={'move_fatigue_counter_in':'normal_ssm_fatigue_level',
                                              'move_fatigue_counter_out':'normal_ssm_fatigue_level',
                                              'move_room_to_find':'room_to_find'})

            smach.StateMachine.add('TRACK_BALL', TrackBall(),
                                   transitions={'tired':'sleepy_robot',
                                                'registered':'MOVE',
                                                'room_founded':'MOVE',
                                                'ball_lost':'MOVE'},
                                   remapping={'track_ball_fatigue_counter_in':'normal_ssm_fatigue_level',
                                              'track_ball_fatigue_counter_out':'normal_ssm_fatigue_level',
                                              'track_room_to_find':'room_to_find'})



        smach.StateMachine.add('NORMAL', normal_sub_sm,
                               transitions={'sleepy_robot':'REST',
                                            'time_to_play':'PLAY'},
                               remapping={'normal_ssm_fatigue_level':'fatigue_level',
                                          'normal_ssm_fatigue_level':'fatigue_level'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'NORMAL'},
                               remapping={'rest_fatigue_counter_in':'fatigue_level',
                                          'rest_fatigue_counter_out':'fatigue_level'})

        find_sub_sm = smach.StateMachine(outcomes=['sleepy_robot', 'restart_playing'],
                              output_keys=['find_sub_sm_fatigue_level'],
                              input_keys=['find_sub_sm_fatigue_level', 'find_sub_sm_room_to_find'])

        find_sub_sm.userdata.explore_time = 0
        with find_sub_sm :


            smach.StateMachine.add('EXPLORE', Explore(),
                                   transitions={'track':'TRACK_BALL',
                                                'stop_exploring':'restart_playing'},
                                   remapping={'start_explore_time_in': 'explore_time',
                                              'start_explore_time_out':'explore_time'})

            smach.StateMachine.add('TRACK_BALL', TrackBall(),
                                   transitions={'tired':'sleepy_robot',
                                                'registered':'EXPLORE',
                                                'room_founded':'restart_playing',
                                                'ball_lost':'EXPLORE'},
                                   remapping={'track_ball_fatigue_counter_in':'find_sub_sm_fatigue_level',
                                              'track_ball_fatigue_counter_out':'find_sub_sm_fatigue_level',
                                              'start_explore_time_out':'explore_time',
                                              'track_room_to_find':'find_sub_sm_room_to_find'})

        smach.StateMachine.add('FIND', find_sub_sm,
                              transitions={'sleepy_robot':'REST',
                                           'restart_playing':'PLAY'},
                              remapping={'find_sub_sm_fatigue_level':'fatigue_level',
                                         'find_sub_sm_fatigue_level':'fatigue_level',
                                         'find_sub_sm_room_to_find':'room_to_find'})


        smach.StateMachine.add('PLAY', Play(),
                                           transitions={'find':'FIND',
                                                        'stop_play':'NORMAL',
                                                        'tired':'REST'},
                                           remapping={'play_fatigue_counter_in':'fatigue_level',
                                                      'play_fatigue_counter_out':'fatigue_level',
                                                      'start_play_time_in': 'start_play_time',
                                                      'start_play_time_out':'start_play_time',
                                                      'play_room_to_find':'room_to_find'})




    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('robot_behavior_state_machine', robot_behaviors, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = robot_behaviors.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
