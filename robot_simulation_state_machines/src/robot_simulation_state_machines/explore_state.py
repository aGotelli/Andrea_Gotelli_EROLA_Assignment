#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file explore_state.py
#   \brief This file contains the declaration of the class describing the Explore state.
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
import actionlib
from explore_lite.msg import ExploreAction, ExploreGoal
import robot_simulation_state_machines.image_processing as imp

##
#   \brief max_explore_time defines the maximum amount of time the robot will stay in the Explore state.
max_explore_time = 300

##
#   \class Explore
#   \brief This class defines the state of the state machine corresponding to the robot looking for a specific room.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Find state.
#   As part of the smach class, this class has the member function execute() providing the intended
#   behavior. For more details about the content of this class, see the member function documentation.
#
class Explore(smach.State):
    ##
    #   \brief The __init__ constructor initializes the state outcome and input output keys as well as a declaration
    #   of an internal subscriber to the action Service provided in explore_lite.
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['track', 'stop_exploring'],
                                 input_keys=['start_explore_time_in'],
                                 output_keys=['start_explore_time_out'])
            self.explore_client = actionlib.SimpleActionClient('explore_action_server',ExploreAction)
            self.count = 10
    ##
    #   \brief execute is the member function executing the state behavior
    #   \param userdata is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This function is executed with the only goal of calling a action service. This action service is provided by the
    #   explore_lite package and makes the robot moving in the non explored edges of the generated map. This state
    #   has only two possible outcomes. The first one is 'track' which is fired when a ball is detected. This outcome makes the robot
    #   to track the ball with the TrackBall state. The second one is 'stop_exploring', which is returned when the time
    #   the robot passed in the Explore state is bigger then the maximum value.
    #   In order to handle the time, the userdata has a spacific imput and output key. When entering the state for the first time
    #   the of the start_explore_time_in will be 0. In this case, the userdata start_explore_time_out is initialized to the current time. When
    #   leaving the state with the 'stop_exploring', the userdata start_explore_time_out is set to 0.
    #   In this way, the next time entering this state the value will be re-initialized. In the second case, when
    #   leaving the state with the 'track' transition, the value is left as it is, so the next time entering this state (after having
    #   reached the ball) is it possible to account for the whole time passed in Explore.
    #   Finally, if the map is all explored, it leave the state with the 'stop_exploring' transition.
    #
    def execute(self, userdata):
        if userdata.start_explore_time_in != 0 :
            elapsed_time = rospy.Time.now().to_sec() - userdata.start_explore_time_in
            print("Time passed in Explore state:", "%.3f"%elapsed_time, "[s]")
        else :
            print("Time passed in Explore state:", 0, "[s]")
            #   Initialize the time if the first time in Explore
            userdata.start_explore_time_out = rospy.Time.now().to_sec()
            self.count = 10
        time_in_explore = rospy.Time.now().to_sec() - userdata.start_explore_time_in
        #   initialize creating an action service request
        self.explore_client.wait_for_server()
        goal = ExploreGoal()
        print("Start exploring")
        self.explore_client.send_goal(goal)
        #   Loop until stopping criteria is reached
        while not rospy.is_shutdown():
            #   Check if the map il totally explored
            state = self.explore_client.get_state()
            if state == 4 :
                print("The map is explored!")
                self.explore_client.cancel_all_goals()
                rospy.sleep(0.5)
                return 'stop_exploring'
            if time_in_explore >= max_explore_time:
                #   Reset time in play as we stop
                userdata.start_explore_time_out = 0
                self.explore_client.cancel_all_goals()
                rospy.sleep(0.5)
                return 'stop_exploring'
            if imp.ball_detected:
                print("A new ball has been detected!")
                self.explore_client.cancel_all_goals()
                rospy.sleep(0.5)
                return 'track'
            #   Check the time in explore
            time_in_explore = rospy.Time.now().to_sec() - userdata.start_explore_time_in
            percentage = 100*time_in_explore/max_explore_time
            #   Print the exploration percentage
            if percentage >= self.count and not self.count == 100:
                if percentage - self.count < 10:
                    print(self.count, "% of exploration time")
                self.count = self.count + 10

