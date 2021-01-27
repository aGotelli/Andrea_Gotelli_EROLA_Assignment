# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file find_state.py
#   \brief This file contains the declaration of the class describing the Find state.
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
#   \class Find
#   \brief This class defines the state of the state machine corresponding to the robot looking for a specific room.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Find state.
#   As part of the smach class, this class has the member function execute() providing the intended
#   behavior. For more details about the content of this class, see the member function documentation.
#
class Find(smach.State):
    ##
    #   \brief The __init__ constructor initializes the state outcome and input output keys as well as a declaration
    #   of an internal subscriber to the action Service provided in Explore Lite.
    def __init__(self):
            smach.State.__init__(self, outcomes=['track'])
            self.explore_client = actionlib.SimpleActionClient('explore_action_server',ExploreAction)
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This function is executed with the only goal of calling a action service. This action service is provided by the
    #   explore_lite package and makes the robot moving randomly in the non explored edges of the generated map. This state
    #   has only one possible outcome that is 'track' which is fired when a ball is detected. This outcome makes the robot
    #   to track the ball with the TrackBall state.
    #
    def execute(self, userdata):
        global ball_detected
        #   initialize creating an action service request
        self.explore_client.wait_for_server()
        goal = ExploreGoal()
        self.explore_client.send_goal(goal)
        #   Loop until stopping criteria is reached
        while not rospy.is_shutdown():
            if imp.ball_detected:
                print("A new ball has been detected!")
                self.explore_client.cancel_all_goals()
                return 'track'
