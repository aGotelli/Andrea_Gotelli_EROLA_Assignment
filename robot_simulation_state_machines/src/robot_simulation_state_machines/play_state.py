#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file play_state.py
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
import rospy
import smach
import smach_ros
import random
from geometry_msgs.msg import Pose
from robot_simulation_messages.srv import PersonCommand
from robot_simulation_state_machines.move_state import isTired
from robot_simulation_state_machines.reach_goal import reachPosition

import robot_simulation_state_machines.image_processing as imp


##
#   \brief max_play_time defines the maximum amount of time the robot will stay in the Play behavior.
max_play_time = 600

##
#   \class Play
#   \brief This class defines the state of the state machine corresponding to the robot interacting with the person.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is a substate of the states Find. In this state the robot moves towards
#   the ball that has beed detected. This is done by simply publishing the twist that has been
#   computed in the callback. This class is implemented in two state machines: the one for the Normal behavior and the one for
#   the Play behavior. As a result, the outcomes available/used depend on the context.
#   As part of the smach class, this class has the member function execute() providing the intended behavior.
#   For more details about the content of this class, see the member function documentation.
#
class Play(smach.State):
    ##
    #   \brief __init__ is the constructor for the class.
    #
    #   This constructor initializes the outcomes and the input outout keys for this state.
    #   Moreover, it initializes the service client which requests for the person decision.
    #
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['find','tired','stop_play'],
                             input_keys=['play_fatigue_counter_in', 'start_play_time_in'],
                             output_keys=['play_fatigue_counter_out','play_room_to_find', 'start_play_time_out'])

        self.person_srv_client = rospy.ServiceProxy('/person_decision', PersonCommand)
        self.person_pose = Pose()
        self.person_pose.position.x = -5
        self.person_pose.position.y = 8

    ##
    #   \brief execute This function performs the behavior for the state
    #   \param userdata Is the structure containing the data shared among states, it is used
    #   to pass the level of fatigue among states.
    #   \return a string consisting of the state outcome
    #
    #   This function simulates the robot interacting with the person.
    #   When executing this memeber function, first the time to remain in this state is initialized. After that,
    #   the function consists in a loop which is interrupted only with some instance checking. The first thing that is
    #   checked is that the time elapsed since the robot is in this state is bigger than a given threshold. In this case,
    #   the return key 'stop_play' brings the robot back in the Normal behavior. It then orders the robot to reach the person
    #   postion, which is stored as a memeber of the class, with a call of the blocking version of reachPosition().
    #   Once the robot has reached the person position, it then calls the service for obtaining an order from the person: the
    #   specification of a room to reach. Once this information is obtained then it looks if the corresponding room is the list
    #   has been registered, in which case it will command the robot to reach the associated position (again with a call of the
    #   blocking version of reachPosition()). The choice of using the blocking version is for prevent the robot of being "distracted"
    #   by the detection of other balls in the environment. On the other hand, if the room is not yet registed, it returns the
    #   outout key 'find' in order to switch state to Find and look for the requested room.
    #
    def execute(self, userdata):
        rospy.wait_for_service('/person_decision')
        print("Time passed in Play behavior:", userdata.start_play_time_in, "[s]")
        #   Initialize the time if the first time in play
        if not userdata.start_play_time_in :
            userdata.start_play_time_out = rospy.Time.now().to_sec()
        #   Main loop
        while not rospy.is_shutdown():
            #   Check the time in play
            time_in_play = rospy.Time.now().to_sec() - userdata.start_play_time_in
            if time_in_play >= max_play_time:
                print("Robot is bored of playing")
                #   Reset time in play as we stop
                userdata.start_play_time_out = 0
                want_play = False
                self.person_srv_client(want_play)
                return 'stop_play'
            #   Reach the person position
            print("Reaching person position")
            reachPosition(self.person_pose, wait=True)
            #   Ask for the person choice (which room to reach?)
            print("Waiting for a command from the person")
            want_play = True
            desired = self.person_srv_client(want_play)
            print("Command received: go to the", desired.room)
            available_room = False
            #   Search for the desired room in the rooms list
            for room in imp.rooms_list:
                #   If it is available then reach the associated position
                if room[1].is_registered and room[0] == desired.room :
                    available_room = True
                    print("Going to room ", room[0] )
                    reachPosition(room[1].position,  wait=True)
                    #   Increase level of fatigue
                    userdata.play_fatigue_counter_out = userdata.play_fatigue_counter_in + 1
                    #   Print a log to show the level of fatigue
                    print('Level of fatigue : ', userdata.play_fatigue_counter_in)
                    #   Check is the robot is tired
                    if isTired(userdata.play_fatigue_counter_in) :
                        #   Print a log to inform about the fact that the robot is tired
                        print('Robot is tired of moving...')
                        #   Reset time in play as we stop
                        userdata.start_play_time_out = 0
                        want_play = False
                        self.person_srv_client(want_play)
                        #   Return 'tired' to change the state
                        return 'tired'
            if not available_room :
                #   If the flag is false then the room is not available yet and thus is the case of look for it
                print("The room", desired.room, "is not available yet...")
                userdata.play_room_to_find = desired.room
                print("Current time in play :", userdata.start_play_time_in)
                return 'find'
