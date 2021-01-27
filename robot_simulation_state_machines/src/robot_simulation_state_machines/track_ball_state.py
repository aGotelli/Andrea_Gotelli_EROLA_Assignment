#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file track_ball_state.py
#   \brief This file contains the class declaration for the state TrackBall
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
#


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
#   \brief maximum_dead_time defines the maximum time to wait for seeing a ball before returning into the MOVE state.
maximum_dead_time = 0

##
#   \brief robot_controller is the controller which directly controls the robot velocity
robot_controller = None


##
#   \brief correction_twist is the twist computed to keep the robot away from the walls, avoiding crash or dangerous situations.
correction_twist = Twist()



##
#   \brief laserReadingCallback callback for the messages received from the laser sensor
#   \param msg is the message containing the laser reading
#
#   This function uses the reading from the laser scan selecting only some specific regions. In fact, only two regions of the
#   laser scan are considered. Specifically, the region corresponding to the left/front left of the robot and the one corresponding
#   to the right/front right of the robot. Comparing the lowest value with an hard threshold, a twist is generated in order to
#   rotate the robot from the wall. For example, if the laser scan reports a wall in roximity on the left of the robot, then a twist
#   is computed in order to make the robot slightly rotate in the clockwise direction. This computation is likely to compete with
#   the twist needed to reach the ball. In order to not overconstrain the system, the computation is performed only if the ball
#   is not marked to be close to the robot. In fact, once the robot is close to the ball, it may also be close to some walls but in
#   this case there is no risk of collision. Moreover, the robot must be free to move in order to complete the traking.
#
def laserReadingCallback(msg):
    global correction_twist

    #   Declare
    regions = {
        'right':  min(min(msg.ranges[0:287]), 10),
        'left':   min(min(msg.ranges[432:719]), 10)
    }
    #   Set as null twist preventively (may not be changed anymore)
    correction_twist = Twist()
    #   Proceed only if the robot is still "far" from the ball to avoid overconstrain the traking task
    if not imp.ball_is_close:
        #   If the wall is too close on the left, turn slightly on the right
        if regions['left'] < 0.8 :
            #   Turn slightly on the right
            correction_twist.angular.z = - 0.001
        #   If the wall is too close on the right, turn slightly on the left
        elif regions['right'] < 0.8 :
            #   Turn slightly on the left
            correction_twist.angular.z = 0.001




##
#   \class TrackBall
#   \brief This class defines the state of the state machine corresponding to the robot moving towards the detected ball.
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is a substate of the states Find. In this state the robot moves towards
#   the ball that has beed detected. This is done by simply publishing the twist that has been
#   computed in the callback. This class is implemented in two state machines: the one for the Normal behavior and the one for
#   the Play behavior. As a result, the outcomes available/used depend on the context.
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
    #   This funtion may be executed in the Normal behavior or the Play behavior. Depending on the
    #   the context in which it is executed, some part may or may not be of interest.
    #   From a general point of view, this function makes the robot move towards the detected ball.
    #   To do so, it publishes the geometry_msgs/Twist message defined in the imageReceived() callback.
    #   It makes the robot to follow the ball until the ball is consider close to the robot and this
    #   last one is also still. To satifly the first condition, the value of the dedicated boolean:
    #   ball_is_close is checked. Secondly, for the other condition, both the linear and angular
    #   speeds are consider to determine if the robot is still moving. If the ball is close to the
    #   robot and the robot is not significantly moving than the ball is considered as reached.
    #   Once the ball has been reached, first it increases the fatigue counter as a new motion has been
    #   completed, then it checks that the robot has not reached the fatigue threshold. In this case it retrieve
    #   from the userdata variable if there was a specified room to find. When there is no specification on the room to
    #   find, this means that the this class is executed as a substate of the Normal behavior. As a result, the robot should
    #   just return 'registed' that is the outcome specified in the sub state machine for this behavior. This outcome
    #   will bring the substate machine back in the Move state.
    #   On the other hand, if there is a specified room to find, this means that the execution of this function is within the
    #   Play behavior, and the robot was looking for a room missing on the list. In this case, the outcome of this function depends
    #   wheter, or not, the robot has found the room of interest. If the search was successfull and the robot has found the interested
    #   room, then the outcome 'room_founded' will bring the substate machine in the Interact state. On the other hand, if another
    #   room is found, then the outcome 'registered' informs that the new room has been registered and brings the substate machine
    #   back in the Find state.
    #   Finally, if the balls disappears from the camera field of view, it changes the state
    #   with the transition 'ball_lost' in order to search trigger the corresponding state depending in which behavior this function
    #   is executed.
    #
    def execute(self, userdata):
        global correction_twist
        global maximum_dead_time
        global robot_controller
        #   Loop until exit condition
        while not rospy.is_shutdown():
            #   First check if too much time has passed since last detection
            if imp.time_since >= maximum_dead_time :
                #   Exit wit the corresponding statement
                return 'ball_lost'
            #   Combine the two twist in order to reach the ball while avoiding the walls
            resulting_twist = Twist()
            resulting_twist = imp.robot_twist
            resulting_twist.angular.z = resulting_twist.angular.z + correction_twist.angular.z
            robot_controller.publish(resulting_twist)
            #   Proceed only if close to the ball
            if imp.ball_is_close:
                # Evaluate if the robot is still
                vels = math.sqrt( (imp.robot_twist.linear.x*imp.robot_twist.linear.x) + (imp.robot_twist.angular.z*imp.robot_twist.angular.z) )
                if vels <= 0.05:
                    #   Set to false for avoid bug
                    imp.ball_is_close = False
                    imp.ball_detected = False
                    #   Make sure the robot stays still
                    null_twist = Twist()
                    robot_controller.publish(null_twist)
                    print("Ball Reached")
                    #   Increment the counter for the fatigue as the robot has moved
                    userdata.track_ball_fatigue_counter_out = userdata.track_ball_fatigue_counter_in + 1
                    print('Level of fatigue : ', userdata.track_ball_fatigue_counter_in)
                    #   Check if robot is tired now
                    if isTired(userdata.track_ball_fatigue_counter_in) :
                        #   Print a log to inform about the fact that the robot is tired
                        print('Robot is tired of moving...')
                        #   Exit with the right condition
                        return 'tired'
                    #   Retrieve what was the missing room to look for
                    room_to_find = userdata.track_room_to_find
                    #   Check wheter there is a room to look for
                    if not room_to_find :
                        #   Return the corresponding change of state
                        return 'registered'
                    else :
                        #   In the case, first retrieve the room the robot has just registed
                        founded_room = imp.registerRoom(rp.robot_pose)
                        #   Then compare it with the room to find, and act as consequence
                        if founded_room == room_to_find:
                            print("The ", founded_room, " is reached")
                            return 'room_founded'
                        else:
                            print("Found ", founded_room, " but the goal is to find ", room_to_find)
                            return 'registered'
