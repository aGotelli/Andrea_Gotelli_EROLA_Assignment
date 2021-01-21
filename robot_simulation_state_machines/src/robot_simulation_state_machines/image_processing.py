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
radius_threshold = 100


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
    global ball_detected
    global ball_is_close
    global robot_twist
    global time_since
    global last_detection
    global max_speed
    global detection_time

    #   Update last detection timestamp
    time_since = rospy.Time.now().to_sec() - last_detection

    #### direct conversion to CV2 ####
    np_arr = np.frombuffer(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    (cnts, circle_color) = findBallIn(hsv)

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
            robot_twist.angular.z = -0.005*(center[0]-250)
            robot_twist.linear.x = 0.02*( (radius_threshold*1.10)-radius)

            scale = 1
            time_from_first_detection = rospy.Time.now().to_sec() - detection_time
            #   Increase linearly for 3 seconds to avoid soaring
            if ( time_from_first_detection <= 3.0 ):
                scale = time_from_first_detection/3
            #   Limit the robot maximum linear speed
            if abs(robot_twist.linear.x) > max_speed :
                robot_twist.linear.x = np.sign(robot_twist.linear.x)*max_speed
            #   Increase linearly
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
            else:
                ball_is_close = False
    else:
        ball_detected = False
        robot_twist = Twist()

    cv2.imshow('robot_camera', image_np)
    cv2.waitKey(1)





class Ball:
    def __init__(self, lower_range_, upper_range_, circle_color_):
        self.lower_range = lower_range_
        self.upper_range = upper_range_
        self.in_cam_view = False
        self.position = Pose()
        self.circle_color = circle_color_
        self.is_registered = False


blue_ball    = Ball( lower_range_=(100, 50, 50), upper_range_=(130, 255, 255),   circle_color_=(255, 0, 0)    )
red_ball     = Ball( lower_range_=(0, 50, 50),   upper_range_=(5, 255, 255),     circle_color_=(0, 0, 255)    )
green_ball   = Ball( lower_range_=(50, 50, 50),  upper_range_=(70, 255, 255),    circle_color_=(0, 255, 0)    )
yellow_ball  = Ball( lower_range_=(25, 50, 50),  upper_range_=(35, 255, 255),    circle_color_=(0, 251, 253)  )
magenta_ball = Ball( lower_range_=(125, 50, 50), upper_range_=(150, 255, 255),   circle_color_=(10, 135, 255) )
black_ball   = Ball( lower_range_=(0, 0, 0),     upper_range_=(5,50,50),         circle_color_=(0, 0, 0)      )

rooms_list = np.array([ ( 'Entrance',   blue_ball       ),
                        ( 'Closet',     red_ball        ),
                        ( 'LivingRoom', green_ball      ),
                        ( 'Kitchen',    yellow_ball     ),
                        ( 'Bathroom',   magenta_ball    ),
                        ( 'Bedroom',    black_ball      )])


def findBallIn(hsv_image, index = 0):
    global ball_detected
    global ball_is_close
    global rooms_list

    lowerLimit = rooms_list[index][1].lower_range
    upperLimit = rooms_list[index][1].upper_range
    circle_color = rooms_list[index][1].circle_color

    mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    #   If ball is marker ad registerd we don't need to porcess it again
    if rooms_list[index][1].is_registered :
        rooms_list[index][1].in_cam_view = False
        ball_is_close = False
        ball_detected = False
        cnts = np.array([])
        circle_color = (255, 255, 255)
        return cnts, circle_color
    else :
        if len(cnts) > 0:
            #   Mark the ball as currently seen
            rooms_list[index][1].in_cam_view = True
            return cnts, circle_color
        elif rooms_list[index][1].in_cam_view :
            #   Set back the ball as not currently in the camera field of view
            rooms_list[index][1].in_cam_view = False

    if (index == len(rooms_list) - 1):
        circle_color = (255, 255, 255)
        return cnts, circle_color

    index = index + 1
    return findBallIn(hsv_image, index)




def registerRoom(robot_pose):
    global rooms_list
    for index in range( len(rooms_list) ):
        if rooms_list[index][1].in_cam_view :
            rooms_list[index][1].is_registered = True
            rooms_list[index][1].position = robot_pose
            print("The ", rooms_list[index][0], " is now available.")
            return rooms_list[index][0]


def sarCallback(req):
    global rooms_list
    av_room_list =[]
    av_room_list.append("Available room(s): ")
    for index in range( len(rooms_list) ):
        if rooms_list[index][1].is_registered :
            av_room_list.append(rooms_list[index][0])
    if len(av_room_list) == 1:
        av_room_list.append("None")
    return ShowAvailableRoomsResponse(av_room_list)

