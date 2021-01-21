import rospy
import smach
import smach_ros
import actionlib
from explore_lite.msg import ExploreAction, ExploreGoal
import robot_simulation_state_machines.image_processing as imp


class Find(smach.State):
    ##
    #   \brief The __init__ constructor initializes the state outcome and input output keys
    def __init__(self):
            smach.State.__init__(self, outcomes=['track'])
            self.explore_client = actionlib.SimpleActionClient('explore_action_server',ExploreAction)
    ##
    #   \brief  The member function executing the state behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #
    def execute(self, userdata):
        global ball_detected
        self.explore_client.wait_for_server()

        goal = ExploreGoal()

        self.explore_client.send_goal(goal)
        while not rospy.is_shutdown():
            if imp.ball_detected:
                print("A new ball has been detected!")
                self.explore_client.cancel_all_goals()
                return 'track'
