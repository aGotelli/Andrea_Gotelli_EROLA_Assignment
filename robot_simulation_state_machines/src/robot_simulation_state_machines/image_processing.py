#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8
## @package robot_simulation_state_machines
#   \file image_processing.py
#   \brief This file contains the functions needed to obtain information from the camera video
#   \author Andrea Gotelli
#   \version 0.2
#   \date 22/10/2020
#
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
#   This file contains function and global variables declarations that are required for the image processing. The functions
#   are divided in the one strictly related to the image processing and the one more oriented to the robot control.
#

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from robot_simulation_messages.srv import ShowAvailableRooms, ShowAvailableRoomsResponse


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
#   \brief Is the maximum linear velocity included for safe navigation.
max_speed = 0

##
#   \brief Is the time passed from the moment the ball appeared in the camera view,
#           it incresed as long as the ball remains detected.
detection_time = 0.0

##
#   \brief Is the value for the radius of the circle containing the ball to consider as a
#           reference value to assume that the ball is "close" to the robot.
radius_threshold = 70


##
#   \brief  imageReceived   Is the subscriber callback for the images published by the camera.
#   \param  ros_data Is the image received, which is of type sensor_msgs/CompressedImage.
#
#   This funtion first converts the image from the type received from the camera to an image which can be
#   processed by OpenCV. It then calls a separate function for the image processing: findBallIn(). It relies directly
#   on the results of the image processing requiring the center and the radius of the enclosing circle as
#   well as the image with the enclosing circle drawn. While the image is directly displayed, the center and
#   radius of the circle are used to compute the twist required to move the robot a certain distance to the ball,
#   which we also wanto to center in the image of the camera.
#   In the computation of the linear velocity, it limits the maximum value in order to avoid dangerous soaring.
#   The limitation must be stronger in the first instants of the of the motion and less invasive later.
#   For this reason the twist is linearly incremented from the 20% to its 100% in the first 3 seconds of motion.
#   It also changes the value or the gobal boolean ball_detected to True.
#   On the other hand,if the radius is equal or bigger than a given threshold, it assumes that the ball is
#   close to the robot. This will be one of the two key points in order to establish whether the ball
#   as been reached or not.
#   Finally, in the case that none of the previusly happened, it sets both the global boolean ball_detected
#   and ball_is_close to False.
#
def imageReceived(ros_data):
    global ball_detected
    global ball_is_close
    global robot_twist
    global time_since
    global last_detection
    global max_speed
    global detection_time

    #   Update last detection timestamp
    time_since = rospy.Time.now().to_sec() - last_detection

    #   Direct conversion to CV2
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    #   Process the image to find one of the described balls
    (image_np, center, radius, succeeded) = findBallIn(hsv, image_np)
    if succeeded :
        #   Check is detection happened after some non-detection time
        if not ball_detected:
            detection_time  = rospy.Time.now().to_sec()
        #   Compute the twist to center the ball in the camera frame and bring the
        #   enclosing circle to a certain radius.
        robot_twist = Twist()
        robot_twist.angular.z = -0.005*(center[0]-250)
        robot_twist.linear.x = 0.02*( (radius_threshold*1.10)-radius)

        scale = 1
        time_from_first_detection = rospy.Time.now().to_sec() - detection_time
        #   Increase linearly for 3 seconds to avoid soaring
        if ( time_from_first_detection <= 3.0 ):
            scale = time_from_first_detection/3
        if scale < 0.2:
            #   Ensure 20% of the computed speed
            scale = 0.2
        #   Limit the robot maximum linear speed
        if abs(robot_twist.linear.x) > max_speed :
            robot_twist.linear.x = np.sign(robot_twist.linear.x)*max_speed
        #   Increase linearly
        robot_twist.linear.x = robot_twist.linear.x*scale
        robot_twist.angular.z = robot_twist.angular.z*scale
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
        else:
            ball_is_close = False
    else:
        ball_detected = False
        robot_twist = Twist()
    #   Proceed in displaying the image
    cv2.imshow('robot_camera', image_np)
    cv2.waitKey(1)




##
#   \class Ball
#   \brief Defines the caracteristic of a ball associated to a room
#
#   This class contains a list of memebers which allows to easily couple a ball with a room. Each instance of
#   this class is associated to a string which uniquely defines a room. This class contains memebers for the
#   color range of a ball, it's position in the map and some additional information like if the ball is currently
#   in the camera field of view, if has been registerd and the color of its enclosing circle.
class Ball:
    ##
    #   \brief __init__ initialises the memebers of the class.
    #   \param lower_range_ is the lower RGB color range.
    #   \param upper_range_ is the upper RGB color range.
    #   \param circle_color_ is the color for the enclosing circle.
    #
    def __init__(self, lower_range_, upper_range_, circle_color_):
        self.lower_range = lower_range_
        self.upper_range = upper_range_
        self.in_cam_view = False
        self.position = Pose()
        self.circle_color = circle_color_
        self.is_registered = False

##
#   \brief blue_ball is the instance of the blue ball.
blue_ball    = Ball( lower_range_=(100, 50, 50), upper_range_=(130, 255, 255),   circle_color_=(255, 0, 0)    )
##
#   \brief red_ball is the instance of the red ball.
red_ball     = Ball( lower_range_=(0, 50, 50),   upper_range_=(5, 255, 255),     circle_color_=(0, 0, 255)    )
##
#   \brief green_ball is the instance of the green ball.
green_ball   = Ball( lower_range_=(50, 50, 50),  upper_range_=(70, 255, 255),    circle_color_=(0, 255, 0)    )
##
#   \brief yellow_ball is the instance of the yellow ball.
yellow_ball  = Ball( lower_range_=(25, 50, 50),  upper_range_=(35, 255, 255),    circle_color_=(0, 251, 253)  )
##
#   \brief magenta_ball is the instance of the magenta ball.
magenta_ball = Ball( lower_range_=(125, 50, 50), upper_range_=(150, 255, 255),   circle_color_=(10, 135, 255) )
##
#   \brief black_ball is the instance of the black ball.
black_ball   = Ball( lower_range_=(0, 0, 0),     upper_range_=(10,50,50),         circle_color_=(0, 0, 0)     )

##
#   \brief rooms_list defines a list of ball with the associated room name.
rooms_list = np.array([ ( 'Entrance',   blue_ball       ),
                        ( 'Closet',     red_ball        ),
                        ( 'LivingRoom', green_ball      ),
                        ( 'Kitchen',    yellow_ball     ),
                        ( 'Bathroom',   magenta_ball    ),
                        ( 'Bedroom',    black_ball      )])




##
#   \brief findBallIn performs image processing in order to find one of the described balls in the image.
#   \param hsv_image is the image that will be proceed
#   \param image_np is the non processed image
#   \param index is at which element of the list of ball look for
#
#   This function is a recursive calling function which aims to loop into the room list and search for the respective ball
#   in the image.
#   It performs the passages of a classic image processing applying some lower and upper color ranges accordingly with the
#   ball to find. If some contours have been found, only the bigger contour is kept and then the image processing continues
#   computing the center and radius of the minimum enclosing circle. The radius is compared with a minimum threshold to
#   establish if the founded contour could be a ball or not. In this phase it takes into account that the radius
#   slightly decreases when the ball is centered in the image. In case of a positive response, the function returns: the center
#   and the radius of the minimum enclosing circle and the non processed image with the enclosing circle drawn on it.
#   Finally, some instance checking is also performed. In the case the room corresponding to the current iteration is
#   already registered, it not necessary to do any further processing and a recursive call is executed. Moreover, this
#   function handles the situation where the index has reached the end of the list of room array: returning a False boolean
#   which interromps the computations in imageReceived().
#
def findBallIn(hsv_image, image_np, index = 0):
    global ball_detected
    global ball_is_close
    global rooms_list
    #   First check if we are out of range
    if (index >= len(rooms_list)):
        center = (0, 0)
        radius = 0.0
        succeeded = False
        return image_np, center, radius, succeeded
    #   Define the ball as not in cam view by default
    rooms_list[index][1].in_cam_view = False
    #   If ball is marker ad registerd we don't need to process it again
    if rooms_list[index][1].is_registered :
        #   Continue with a recursive call
        index = index + 1
        return findBallIn(hsv_image, image_np, index)
    #   Obtain the RGB ranges for the ball associated with the current room
    lowerLimit = rooms_list[index][1].lower_range
    upperLimit = rooms_list[index][1].upper_range
    circle_color = rooms_list[index][1].circle_color
    #   Perform the contour detection
    mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    #   If at least one contour is found continue with the image processing
    if len(cnts) > 0:
        #   Mark the ball as currently seen
        rooms_list[index][1].in_cam_view = True
        #   Find the largest contour in the mask, then use it to compute the minimum
        #   enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        #   Define the radius minimum size
        threshold = 10
        #   Check if the radius of the enclosing circle is bigger than the threshold.
        #   Also consider that when centering the radius descreases
        if radius >= threshold or (ball_detected and radius >= 0.80*threshold):
            #   Draw the circle and centroid on the frame, then update the list of tracked points
            cv2.circle(image_np, (int(x), int(y)), int(radius), circle_color, 2)
            cv2.circle(image_np, center, 5, (255, 255, 255), -1)
            succeeded = True
            return image_np, center, radius, succeeded
    #   Continue with a recursive call
    index = index + 1
    return findBallIn(hsv_image, image_np, index)

##
#   \brief registerRoom marks a room as registered with the respective position in the map.
#   \param robot_pose is the current postion of the robot when registering the ball
#   \return the name of the room registerd.
#
#   When this function is called means that the ball in front of the robot is reached and needs to be registerd.
#   To do so, it looks in the list of room for the ball that is in the camera field of view. Once found the
#   ball of interest, it marks it as registerd and saves the the current robot position.
#
def registerRoom(robot_pose):
    global rooms_list
    #   Check which ball is in front of the robot
    for room in rooms_list:
        if room[1].in_cam_view and not room[1].is_registered:
            #   Perform saving
            room[1].is_registered = True
            room[1].position = robot_pose
            print("The ", room[0], " is now available.")
            return room[0]
        elif room[1].in_cam_view :
            return room[0]


start_time = 0

##
#   \brief  sarCallback is a service callback which returns the list of registered rooms.
def sarCallback(req):
    global start_time
    time_elapsed = rospy.Time.now().to_sec() - start_time
    time_elapsed = int(time_elapsed/60)
    global rooms_list
    av_room_list =[]
    av_room_list.append("After " + str(time_elapsed) +" min the available room(s): ")
    for room in rooms_list :
        if room[1].is_registered:
            av_room_list.append(room[0])
    if len(av_room_list) == 1:
        av_room_list.append("None")
    return ShowAvailableRoomsResponse(av_room_list)
