import math
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from robot_simulation_state_machines.reach_goal import reachPosition
from robot_simulation_state_machines.move_state import isTired
import robot_simulation_state_machines.image_processing as imp
import robot_simulation_state_machines.robot_position as rp



##
#   \brief Defines the maximum time to wait for seeing a ball before returning into the MOVE state.
maximum_dead_time = 0

##
#   \brief Definition of the controller which directly controls the robot velocity
robot_controller = None



correction_twist = Twist()
def laserReadingCallback(msg):
    global correction_twist
    regions = {
        'right':  min(min(msg.ranges[0:287]), 10),
        'left':   min(min(msg.ranges[432:719]), 10)
    }
    correction_twist = Twist()
    if not imp.ball_is_close:
        if regions['left'] < 0.8 :
            #   Turn slightly on the right
            correction_twist.angular.z = - 0.001
        elif regions['right'] < 0.8 :
            #   Turn slightly on the left
            correction_twist.angular.z = 0.001




##
#   \class TrackBall
#   \brief This class defines the state of the state machine corresponding to the robot moving towards the detected ball.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is a substate of the state Play. In this state the robot moves towards
#   the ball that has beed detected. This is done by simply publishing the twist that has been
#   computed in the callback.
#   As part of the smach class, this class has the member function execute() providing the intended behavior.
#   For more details about the content of this class, see the member function documentation.
#
class TrackBall(smach.State):
    ##
    #   \brief __init__ is the constructor for the class.
    #
    #   This constructor initializes the outcomes and the input outout keys for this state.
    #
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['tired', 'registered', 'room_founded','ball_lost'],
                                 input_keys=['track_ball_fatigue_counter_in','track_room_to_find'],
                                 output_keys=['track_ball_fatigue_counter_out'])

    ##
    #   \brief execute This function performs the behavior for the state
    #   \param userdata Is the structure containing the data shared among states, it is used
    #   to pass the level of fatigue among states.
    #   \return a string consisting of the state outcome
    #
    #   This funtion makes the robot move towards the detected ball. To do so, it publishes the
    #   geometry_msgs/Twist message defined in the imageReceived() callback.
    #   It makes the robot to follow the ball until the ball is consider close to the robot and this
    #   last one is also still. To satifly the first condition, the value of the dedicated boolean:
    #   ball_is_close is checked. Secondly, for the other condition, both the linear and angular
    #   speeds are consider to determine if the robot is still moving. If the ball is close to the
    #   robot and the robot is not significantly moving than the ball is considered as reached.
    #   Once the ball has been reached, first it increases the fatigue counter as a new motion has been
    #   completed, then it checks that the robot has not reached the fatigue threshold. In this case
    #   it returns 'turn_head' in other to chage the state into the TURN_HEAD_COUNTERCLOCKWISE state, described in the
    #   TurnHeadCounterClockWise class. On the other hand, if the level of fatigue has reached the threshold,
    #   then it returns 'tired' in order to change the state into REST, represented in Rest.
    #   On the other hand, if the balls disappears from the camera field of view, it changes the state
    #   with the transition 'ball_lost' in order to search for the ball in the state TURN_ROBOT, see TurnRobot.
    #   Finally, if the ball has not been detected for a time greater then the maximum_dead_time then
    #   it exit the state returning 'stop_play' changing the state into MOVE, see Move.
    #
    def execute(self, userdata):
        global correction_twist
        global maximum_dead_time
        global robot_controller

        while not rospy.is_shutdown():
            if imp.time_since >= maximum_dead_time :
                return 'ball_lost'
            resulting_twist = Twist()
            resulting_twist = imp.robot_twist
            resulting_twist.angular.z = resulting_twist.angular.z + correction_twist.angular.z
            robot_controller.publish(resulting_twist)
            if imp.ball_is_close:
                # Evaluate if the robot is still
                vels = math.sqrt( (imp.robot_twist.linear.x*imp.robot_twist.linear.x) + (imp.robot_twist.angular.z*imp.robot_twist.angular.z) )
                if vels <= 0.05:
                    print("Ball Reached")
                    #   Increment the counter for the fatigue as the robot has moved
                    userdata.track_ball_fatigue_counter_out = userdata.track_ball_fatigue_counter_in + 1
                    print('Level of fatigue : ', userdata.track_ball_fatigue_counter_in)
                    #   Make sure the robot stays still
                    null_twist = Twist()
                    robot_controller.publish(null_twist)
                    founded_room = imp.registerRoom(rp.robot_pose)
                    #   Set to false for avoid bug
                    imp.ball_is_close = False
                    imp.ball_detected = False
                    if isTired(userdata.track_ball_fatigue_counter_in) :
                        #   Print a log to inform about the fact that the robot is tired
                        print('Robot is tired of moving...')
                        return 'tired'
                    desired_room = userdata.track_room_to_find
                    if desired_room == '':
                        return 'registered'
                    else:
                        if founded_room == desired_room:
                            print("The ", founded_room, " is reached")
                            return 'room_founded'
                        else:
                            print("Found ", founded_room, " but the goal is to find ", desired_room)
                            return 'registered'
