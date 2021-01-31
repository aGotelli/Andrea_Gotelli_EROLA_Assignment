#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file play_state.py
#   \brief This file contains the declaration of the class describing the Play state.
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
#   that is represented here is the Play behavior. Here, the robot moves to the person position and
#   thyen waits for a command. It will then go to the pointed location or start searching for it.
#   As part of the smach class, this class has the member function execute() providing the intended behavior.
#   For more details about the content of this class, see the member function documentation.
#
class Play(smach.State):
    ##
    #   \brief __init__ is the constructor for the class.
    #
    #   This constructor initializes the outcomes and the input outout keys for this state.
    #   Moreover, it initializes the service client which requests for the person decision and the
    #   person position itself.
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
    #   This function simulates the robot playing with the person.
    #   When executing this memeber function, first the time to remain in this state is initialized. After that,
    #   the function consists in a loop which is interrupted only with some instance checking. The first thing that is
    #   checked is that if the time elapsed since the robot is in this state is bigger than a given threshold. In this case,
    #   the return key 'stop_play' brings the robot back in the Normal behavior. If it is not the case, then it orders the robot to reach the person
    #   postion, which is stored as a memeber of the class, with a call of the blocking version of reachPosition().
    #   The last occurence which makes the state to change is the possibility that the robot decides to stop play.
    #   This probability is computed comparing the percentage of the time passed in this state with the maximum time
    #   (augmented by a fixed gain). In this way, the more the robot stays in this behavior the more is likely that he will
    #   decide to stop playing. If the maximum time is not augmented by the gain, it will be impossible to reach the maximum dead time
    #   and thus one outcome will be unused.
    #   Once the robot has reached the person position, it then calls the service for obtaining an order from the person: the
    #   specification of a room to reach. Once this information is obtained then it looks if the corresponding room is the list
    #   has been registered, in which case it will command the robot to reach the associated position (again with a call of the
    #   blocking version of reachPosition()). The choice of using the blocking version is for prevent the robot of being "distracted"
    #   by the detection of other balls in the environment. On the other hand, if the room is not yet registed, it returns the
    #   outout key 'find' in order to switch state to Find and look for the requested room. In any case, when leaving this
    #   state, a call of the service 'person_decision' with a negative value for want_to_play will inform the
    #   person node of this decision, resetting the timer for the next call to play.
    #
    def execute(self, userdata):
        rospy.sleep(1)
        rospy.wait_for_service('/person_decision')
        if userdata.start_play_time_in != 0 :
            elapsed_time = rospy.Time.now().to_sec() - userdata.start_play_time_in
            print("Time passed in Play behavior:", "%.3f"%elapsed_time, "[s]")
        else :
            print("Time passed in Play behavior:", 0, "[s]")
            #   Initialize the time if the first time in play
            userdata.start_play_time_out = rospy.Time.now().to_sec()
        #   Main loop
        while not rospy.is_shutdown():
            #   Check the time in play
            time_in_play = rospy.Time.now().to_sec() - userdata.start_play_time_in
            if time_in_play >= max_play_time:
                print("Played for:", "%.3f"%time_in_play, "[s]... Robot is bored of playing...")
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
                    #   Randomly evaluates if leaving the state
                    augmented_max_time = 1.8*max_play_time
                    percentage = 100*time_in_play/augmented_max_time
                    check = random.randint(0,100)
                    if check < percentage :
                        print("Robot has decided to stop playing!")
                        return 'stop_play'
            if not available_room :
                #   If the flag is false then the room is not available yet and thus is the case of look for it
                print("The room", desired.room, "is not available yet...")
                userdata.play_room_to_find = desired.room
                return 'find'
