#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file interact_state.py
#   \brief This file contains the declaration of the class describing the Interact state.
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
import rospy
import smach
import smach_ros
import random
from geometry_msgs.msg import Pose
from robot_simulation_messages.srv import PersonCommand
from robot_simulation_state_machines.move_state import isTired
from robot_simulation_state_machines.reach_goal import reachPosition

import robot_simulation_state_machines.image_processing as imp














class Interact(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['find','tired','stop_play'],
                             input_keys=['interact_fatigue_counter_in'],
                             output_keys=['interact_fatigue_counter_out','room_to_find'])

        self.person_srv_client = rospy.ServiceProxy('/person_decision', PersonCommand)
        self.time_in_play = 0.0

    def execute(self, userdata):
        min_time = 100
        max_time = 200
        self.time_in_play = random.randint(min_time, max_time)
        init_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            time_elapsed = rospy.Time.now().to_sec() - init_time
            if time_elapsed >= self.time_in_play:
                return 'stop_play'
            person_pose = Pose()
            person_pose.position.x = -5
            person_pose.position.y = 8
            print("Reaching person position")
            reachPosition(person_pose, wait=True)

            rospy.wait_for_service('/person_decision')
            print("Waiting for a command from the person")
            desired = self.person_srv_client()
            print("Command received: go to the", desired.room)
            available_room = False
            for room in imp.rooms_list:
                if room[1].is_registered and room[0] == desired.room :
                    available_room = True
                    print("Going to room ", room[0] )
                    reachPosition(room[1].position,  wait=True)
                    #   Increase level of fatigue
                    userdata.interact_fatigue_counter_out = userdata.interact_fatigue_counter_in + 1
                    #   Print a log to show the level of fatigue
                    print('Level of fatigue : ', userdata.interact_fatigue_counter_in)
                    #   Check is the robot is tired
                    if isTired(userdata.interact_fatigue_counter_in) :
                        #   Print a log to inform about the fact that the robot is tired
                        print('Robot is tired of moving...')
                        #   Return 'tired' to change the state
                        return 'tired'
            if not available_room :
                print("The room", desired.room, "is not available yet...")
                userdata.room_to_find = desired.room
                return 'find'
