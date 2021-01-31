#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file rest_state.py
#   \brief This file contains the declaration of the class describing the Rest behavior.
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
#   This file contains the declaration of the class Rest and the global variable sleep_station which allow to simulate the
#   behavior of the robot going to sleep when tired.
#
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from robot_simulation_state_machines.reach_goal import reachPosition



##
#   \brief Is the position where the robot goes to sleep.
sleep_station = Pose()

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
            smach.State.__init__(self,
                                 outcomes=['rested'],
                                 input_keys=['rest_fatigue_counter_in'],
                                 output_keys=['rest_fatigue_counter_out'])
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This simple member function moves the robot in the sleeping position using the blocking
    #   version of reachPosition(). Once reached the sleep_station it waits for some time
    #   in order to simulate the pet like robot sleeping. It also resets the fatigue level to zero
    #   to let the robot to move and performs his behaviors.
    #
    def execute(self, userdata):
        #   Wait two seconds to avoid collision with action service goal cancellation
        rospy.sleep(2)
        print("Reaching sleeping station..")
        #   Call the service to reach the position corresponding to the sleeping position
        reachPosition(sleep_station, wait=True)
        #   Sleep for some time
        print('Sleeping...')
        rospy.sleep(10)
        print('Woke up!')
        #   Reset the fatigue counter afther the robot is well rested
        userdata.rest_fatigue_counter_out = 0
        #   Return 'rested' to change the state
        return 'rested'
