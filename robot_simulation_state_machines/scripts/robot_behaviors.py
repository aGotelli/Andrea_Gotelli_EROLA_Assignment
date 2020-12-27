#!/usr/bin/env python

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
#
#   \details
#
#   Subscribes to: <BR>
#        camera1/image_raw/compressed
#        joint_neck_position_controller/state
#        odom
#
#   Publishes to: <BR>
#        joint_neck_position_controller/command
#        cmd_vel
#
#   Service : <BR>
#        [None]
#
#   Description :
#
#   This file uses smach libraries to generate two state machines, which are used to control the
#   behaviors of the robot. Specifically, the state machines defines the transition from a state to
#   another using the interface provided by smach. There is a top level state machine defining two
#   of the three main robot behaviors. The last behavior is defined by the states of a sub state
#   machine.
#   The states are defined in the respective classes, and the transistions are automatically performed
#   by the state machine from the smach libraries.
#
# This Python file uses the following encoding: utf-8



# /move_base_simple/goal


import roslib
import rospy
import smach
import smach_ros
import time
import random
import math

# Ros Messages
from sensor_msgs.msg import CompressedImage
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

"""
    Problems:
        -> Se lo vede di lato ed è dietro ad angolo, ci si schianta contro allalgolo e si blocca
        -> Il sistema dovrebbe capire quando il target è irraggiungibile
        -> Dopo aver registrato la palla, il robot forse tornare al target prestabilito oppure un altro random?
        -> If, for some reason, it loses the ball while tracking then its stuck
"""

##
#   \brief Define the width of the arena.
width = 0

##
#   \brief Define the height of the arena.
height = 0

##
#   \brief Is the position where the robot goes to sleep.
sleep_station = Pose()

##
#   \brief Defines the maximum time to wait for seeing a ball before returning into the MOVE state.
maximum_dead_time = 0

##
#   \brief Definition of the position publisher for the robot neck joint
neck_controller = None

##
#   \brief Definition of the controller which directly controls the robot velocity
robot_controller = None

##
#   \brief Definition of the frequency for publishing command to the robot neck
neck_controller_rate = None

##
#   \brief Defines the maximum level of fatigue the robot can andle before going to sleep.
fatigue_threshold = 0

##
#   \brief  It is a global bolean to access the fact that the ball has been detected by the image processing algorithm,
ball_detected = False

##
#   \brief  It is a global boolen to make the state aware of the fact that the robot has reached the ball or not. The information comes from the image processing algorithm
ball_is_close = False

##
#   \brief  It is a global geometry_msgs/Twist message to be accessed by the states that need to publish this command
robot_twist = Twist()

##
#   \brief  Is the amount of time, in second, passed since the last detection of the ball
time_since = 0.0

##
#   \brief  Contains information regarding the last timestamp when the ball was detected
last_detection = 0.0

##
#   \brief Is a global variable containing the current angle of rotation of the neck_joint
neck_angle = 0.0

##
#   \brief Is the maximum linear velocity included for safe navigation.
max_speed = 0

##
#   \brief Is the time passed from the moment the ball appeared in the camera view,
#           it incresed as long as the ball remains detected.
detection_time = 0.0

##
#   \brief Is the value for the radius of the circle containing the ball to consider as a
#           reference value to assume that the ball is "close" to the robot.
radius_threshold = 100

time_before_change_target = 0.0

##
#   \brief Is the robot heading
yaw = 0.0
robot_pose = Pose()

##
#    \brief isTired check the level of fatigue to establish if the robot is "tired"
#    \param level [integer] is the current level of fatigue of the robot.
#    \return a boolen. True if the robot is "tired" false elsewhere.
#
#    This function compares the level of fatigue with and hard threshold which is a parameter
#    eccessible for the user.
#
def isTired(fatigue_level):
    global sleepy_robot
    if fatigue_level >= fatigue_threshold:
        return True
    else :
        return False

##
#    \brief odometryReceived stores the value of the current robot heading
#    \param msg is the robot current odometry
#
#    This function simply process the received odometry message in order to
#    to obtain the current heading of the robot.
#
def odometryReceived(msg):
    global yaw
    global robot_pose

    pose = msg.pose.pose
    robot_pose = msg.pose.pose
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

class ColorDetails:
    def __init__(self, lower, upper, circle, room):
        self.lower = lower
        self.upper = upper
        self.circle = circle
        self.in_cam_view = False
        self.registered = False
        self.pose = Pose()
        self.associated_room = room

#blue = ColorDetails( (210,60,60), (250, 100, 100), (255, 0, 0) )
blue = ColorDetails( (60,30,30), (255,120,80), (255, 0, 0), 'entrance' )
red = ColorDetails( (20,20,100), (100,50,50), (0, 0, 255), 'closet' )
green = ColorDetails( (50, 50, 20), (70, 255, 255), (0, 255, 0), 'living_room' )   #ok
yellow = ColorDetails( (20, 100, 100), (30, 255, 255), (0, 251, 253), 'kitchen' )  #ok
orange = ColorDetails( (5, 50, 50), (15, 255, 255), (10, 135, 255), 'bathroom' )
black = ColorDetails( (0,0,0), (20,20,20), (0, 0, 0), 'bedroom' )  #ok

colors = np.array([('green', green), ('yellow', yellow), ('black', black) ])
#lower_blue = np.array(140,100, 140)
#upper_blue = np.array(150,255,255)

def findBallIn(hsv_image, index = 0):
    global colors

    color = colors[index][0]
    lowerLimit = colors[index][1].lower
    upperLimit = colors[index][1].upper
    circle_color = colors[index][1].circle

    mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    #   If ball is marker ad registerd we don't need to porcess it again
    if colors[index][1].registered :
        colors[index][1].in_cam_view = False
        ball_is_close = False
        ball_detected = False
        cnts = np.array([])
        color = 'none'
        circle_color = (255, 255, 255)
        return cnts, color, circle_color
    else :
        if len(cnts) > 0:
            #   Mark the ball as currently seen
            colors[index][1].in_cam_view = True
            return cnts, color, circle_color
        elif colors[index][1].in_cam_view :
            #   Set back the ball as not currently in the camera field of view
            colors[index][1].in_cam_view = False

    if (index == len(colors) - 1):
        color = 'none'
        circle_color = (255, 255, 255)
        return cnts, color, circle_color

    index = index +1
    return findBallIn(hsv_image, index)


def registerBall():
    global colors

    for ball in colors :
        if ball[1].in_cam_view :
            ball[1].registered = True
            ball[1].pose = robot_pose
            print("The ", ball[1].associated_room , " is now available.")


##
#   \brief  imageReceived   Is the subscriber callback for the images published by the camera. It also performs
#           some image processing.
#   \param  ros_data Is the image received, which is of type sensor_msgs/CompressedImage.
#
#   This funtion first converts the image from the type received from the camera to an imgace which can be
#   processed by OpenCV. Then it performs some image processing first by blurring and filtering the image,
#   then by selecting element of a certain color. The color is indicated in the function itself.
#   After having detected the object of given color, it takes into account only the bigger of them.
#   It evaluates the circle which contains the object, computing the center and the radius.
#   Based on the value of the radius, the function determines either if the robot is close to the
#   ball or if it should move closer to it. In this last case, it computes a geometry_msgs/Twist message
#   containing an adjustement of the heading, for having the ball centered in the image, and a linear
#   velocity to move the robot closer to the ball. In the computation of the linear velocity, it limits the
#   maximum value in order to avoid dangerous soaring. The limitation must be stronger in the first instants of the
#   of the motion and less invasive later. For this reason the twist is linearly incremented from the 0% to the
#   100% of the computed value in the first 3 seconds of motion.
#   It also changes the value or the gobal boolean ball_detected to True.
#   On the other hand,if the radius is equal or bigger than a given threshold, it assumes that the ball is
#   close to the robot. This will be one of the two key points in order to establish whether the ball
#   as been reached or not.
#   Finally, in the case that none of the previusly happened, it sets both the global boolean ball_detected
#   and ball_is_close to False.
#
def imageReceived(ros_data):
    global planning_client
    global ball_detected
    global ball_is_close
    global robot_twist
    global time_since
    global last_detection
    global sleepy_robot
    global max_speed
    global detection_time

    #   Update last detection timestamp
    time_since = rospy.Time.now().to_sec() - last_detection

    #### direct conversion to CV2 ####
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    (cnts, color, circle_color) = findBallIn(hsv)
    #print("Founded : ", color)
    center = None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            #   Check is detection happened after some non-detection time
            if not ball_detected:
                detection_time  = rospy.Time.now().to_sec()

            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(image_np, (int(x), int(y)), int(radius),
                       circle_color, 2)
            cv2.circle(image_np, center, 5, (255, 255, 255), -1)
            robot_twist = Twist()
            robot_twist.angular.z = -0.005*(center[0]-400)
            robot_twist.linear.x = 0.02*(110-radius)

            scale = 1
            time_from_first_detection = rospy.Time.now().to_sec() - detection_time
            #   Increase linearly for 3 seconds to avoid soaring
            if ( time_from_first_detection <= 3.0 ):
                scale = time_from_first_detection/3
            #   Limit the robot maximum linear velocity
            if abs(robot_twist.linear.x) > max_speed :
                robot_twist.linear.x = np.sign(robot_twist.linear.x)*max_speed
            robot_twist.linear.x = robot_twist.linear.x*scale
            #   Finalize detection
            ball_detected = True
            #   Reset time of last detection
            last_detection = rospy.Time.now().to_sec()
            #   Set to zero the time interval
            time_since = 0.0
            #   Check if the ball is reached
            if ( radius >= radius_threshold ):
                #   The robot has reached the ball
                ball_is_close = True
                #robot_twist = Twist()
            else:
                ball_is_close = False
    else:
        ball_detected = False
        robot_twist = Twist()

    cv2.imshow('window', image_np)
    cv2.waitKey(2)




##
#   \brief retrieveNeckAngle is the ros subscriber callback which only saves the current neck_joint rotation angle
#   \param joint_state the value of the interested neck joint
def retrieveNeckAngle(joint_state):
    global neck_angle
    neck_angle = joint_state.set_point






def compEuclidDist(target, current):
    delta_x = target.position.x - current.position.x
    delta_y = target.position.y - current.position.y
    dist = math.sqrt( delta_x*delta_x + delta_y*delta_y )
    return dist




##
#   \class Move
#   \brief This class defines the state of the state machine corresponding to the robot randomly moving
#
#   This class inheritates from smach and it consist of a state in the state machine. The state
#   that is represented here is the Move state. In this state the robot moves around randomly calling
#   the dedicated action service. As part of the smach class, this class has the member function execute()
#   providing the intended behavior. For more details about the content of this class, see the member functions
#   documentation.
#
class Move(smach.State):
    ##
    #   \brief __init__ initializes the class and its members
    #   \param outcomes list of the possible outcomes from this state of the state machine.
    #   \param input_keys list of the possible input for this state (user data shared among states).
    #   \param output_keys list of the possible outputs for this state (user data shared among states).
    #
    #   This member function initializes the state machine state. It follows the conventions for smach.
    #
    def __init__(self):
            smach.State.__init__(self,
                                 outcomes=['tired','tracking'],
                                 input_keys=['move_fatigue_counter_in'],
                                 output_keys=['move_fatigue_counter_out'])
            self.target = Pose()
            self.targetSeemsReacheable = True
            self.prev_dist = 0.0

    def newTarget(self):
        self.targetSeemsReacheable = True
        #   Declare a geometry_msgs/Pose for the random position
        random_ = Pose()
        #   Define the random components (x, y) of this random position
        random_.position.x = random.randint(-width/2, width/2)
        random_.position.y = random.randint(-height/2, height/2)
        #   Call the service to reach this position
        self.target = random_
        self.prev_dist = compEuclidDist(random_, robot_pose)
        print("Prev dist: ", self.prev_dist)
        reachPosition(random_, verbose=True)




    def checkReachability(self, event):
        global robot_pose
        curr_dist = compEuclidDist(self.target, robot_pose)
        print("Prev dist: ", self.prev_dist)
        print("Curr dist: ", curr_dist)
        if curr_dist >= self.prev_dist :
            self.targetSeemsReacheable = False
        else:
            self.prev_dist = curr_dist
            self.targetSeemsReacheable = True

    ##
    #   \brief execute is main member function of the class, containing the intended behavior
    #   \param userdata Is the structure containing the data shared among states.
    #   \return a string consisting of the state outcome
    #
    #   This member function is responsible of simulating the Move behavior for the robot.
    #   First it generates a random position, comanding the robot to reach it with the
    #   non blocking version of reachPosition(). After this initialization, it recursively cheks if the
    #   ball has been detected, in which case it uses the transition 'play' in order to move to the Play
    #   behavior, which starts with the FollowBall state. On the other hand, if the ball is not detected,
    #   it checks wheter the robot has reached the goal, looping in this double instances checking.
    #   In the case the robot has reached the goal, in increases the level of fatigue also comparing it
    #   with the fatigue_threshold with isTired(). In case of a positive response, it returns 'tired' in order
    #   to trigger the Rest behavior.Finally, if the robot is not tired, it generates a random position and
    #   it calls again the non blocking verison of reachPosition().
    #
    def execute(self, userdata):
        global ball_detected
        global planning_client
        global time_before_change_target
        #   Declare a geometry_msgs/Pose for the random position
        random_ = Pose()
        #   Define the random components (x, y) of this random position
        #random_.position.x = random.randint(-width/2, width/2)
        #random_.position.y = random.randint(-height/2, height/2)
        random_.position.x = -6
        random_.position.y = 0
        #   Call the service to reach this position
        reachPosition(random_, verbose=True)
        self.target = random_
        self.prev_dist = compEuclidDist(random_, robot_pose)
        print("Prev dist: ", self.prev_dist)
        timer = rospy.Timer(rospy.Duration(time_before_change_target), self.checkReachability)
        #   Main loop
        while not rospy.is_shutdown():
            #   Check if the person has commanded play
            if ball_detected :
                print("Ball detected stopping the robot!")
                #   Stop the robot right there
                planning_client.cancel_all_goals()
                #   Return 'plying' to change the state
                timer.shutdown()
                return 'tracking'
            else :
                if not self.targetSeemsReacheable:
                    print("Target seems impossible to reach")
                    print("Computing a new target")
                    self.newTarget()
                #   If none of the previous was true, then continue with the Move behavior
                if planning_client.get_result():
                    print("Reached the position!")
                    #   Increment the level of the robot fatigue
                    userdata.move_fatigue_counter_out = userdata.move_fatigue_counter_in + 1
                    #   Print a log to show the level of fatigue
                    print('Level of fatigue : ', userdata.move_fatigue_counter_in)
                    #   Check is the robot is tired
                    if isTired(userdata.move_fatigue_counter_in) :
                        #   Print a log to inform about the fact that the robot is tired
                        print('Robot is tired of moving...')
                        #   Return 'tired' to change the state
                        timer.shutdown()
                        return 'tired'
                    else:
                        print("Continuing to move")
                        self.newTarget()



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
                                 outcomes=['registered'],
                                 input_keys=['track_ball_fatigue_counter_in'],
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
        global ball_detected
        global ball_is_close
        global robot_twist
        global time_since
        global maximum_dead_time
        global robot_controller

        while not rospy.is_shutdown():
            robot_controller.publish(robot_twist)
            if ball_is_close:
                # Evaluate if the robot is still
                vels = math.sqrt( (robot_twist.linear.x*robot_twist.linear.x) + (robot_twist.angular.z*robot_twist.angular.z) )
                if vels <= 0.05:
                    print("Ball Reached")
                    #   Increment the counter for the fatigue as the robot has moved
                    userdata.track_ball_fatigue_counter_out = userdata.track_ball_fatigue_counter_in + 1
                    print('Level of fatigue : ', userdata.track_ball_fatigue_counter_in)
                    #   Make sure the robot stays still
                    null_twist = Twist()
                    robot_controller.publish(null_twist)
                    registerBall()
                    print("Position Registered")
                    #   Set to false for avoid bug
                    ball_is_close = False
                    ball_detected = False
                    rospy.sleep(1)
                    return 'registered'



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



class Find(smach.state)
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
    #
    def execute(self, userdata):


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
    rospy.sleep(4)
    print("Starting the state machines")
    #   Initialize the seed
    random.seed()

    #   Dummy Initialization
    last_detection = rospy.Time.now().to_sec()

    #   Subscribed to the recorded image
    image_subscriber = rospy.Subscriber("camera1/image_raw/compressed",
                                            CompressedImage, imageReceived,  queue_size=1)


    #   Definition of the position publisher for the robot neck joint
    neck_controller = rospy.Publisher('joint_neck_position_controller/command', Float64, queue_size=1)

    #   Definition of the frequency for publishing command to the robot neck
    neck_controller_rate = rospy.Rate(50)

    #   Definition of the subscriber to retrive the neck_joint angle
    neck_joint_subscriber = rospy.Subscriber("joint_neck_position_controller/state",
                                                JointControllerState, retrieveNeckAngle,  queue_size=1)

    #   Definition of the pubisher for the robot velocity
    robot_controller = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #   Definition of the subscriber for the robot Odometry
    sub_odom = rospy.Subscriber('odom', Odometry, odometryReceived)

    #   Retrieve the parameter about the world dimensions
    width = rospy.get_param('/world_width', 20)
    height = rospy.get_param('/world_height', 20)

    #   Retrieve parameters about the sleeping position
    sleep_station.position.x = rospy.get_param('/sleep_x_coord', 0)
    sleep_station.position.y = rospy.get_param('/sleep_y_coord', 0)

    #   Retrieve the parameters of the fatigue threshold and max dead time
    fatigue_threshold = rospy.get_param('/fatigue_threshold', 5)
    maximum_dead_time = rospy.get_param('/maximum_dead_time', 5)

    #   Robot maximum linear velocity
    max_speed = rospy.get_param('/max_speed', 0.5)

    print("fatigue_threshold" , fatigue_threshold)
    print("maximum_dead_time" , maximum_dead_time)

    time_before_change_target = rospy.get_param('/time_before_change_target', 40.0)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['behavior_interface'])
    sm.userdata.fatigue_level = 0

    # Open the container
    with sm:
        # Add states to the container

        #   Create the sub state machine for the normal behavior
        sub_sm = smach.StateMachine(outcomes=['sleepy_robot'],
                                    output_keys=['sub_sm_fatigue_level'],
                                    input_keys=['sub_sm_fatigue_level'])

        with sub_sm:

            smach.StateMachine.add('MOVE', Move(),
                                   transitions={'tired':'sleepy_robot',
                                                'tracking':'TRACK_BALL'},
                                   remapping={'move_fatigue_counter_in':'sub_sm_fatigue_level',
                                              'move_fatigue_counter_out':'sub_sm_fatigue_level'})

            smach.StateMachine.add('TRACK_BALL', TrackBall(),
                                   transitions={'registered':'MOVE'},
                                   remapping={'track_ball_fatigue_counter_in':'sub_sm_fatigue_level',
                                              'track_ball_fatigue_counter_out':'sub_sm_fatigue_level'})



        smach.StateMachine.add('NORMAL', sub_sm,
                               transitions={'sleepy_robot':'REST'},
                               remapping={'sub_sm_fatigue_level':'fatigue_level',
                                          'sub_sm_fatigue_level':'fatigue_level'})

        smach.StateMachine.add('REST', Rest(),
                               transitions={'rested':'NORMAL'},
                               remapping={'rest_fatigue_counter_in':'fatigue_level',
                                          'rest_fatigue_counter_out':'fatigue_level'})



    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('robot_behavior_state_machine', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
