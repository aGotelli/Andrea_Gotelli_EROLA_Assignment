def init():
    global width, height
    global sleep_station
    global maximum_dead_time
    global robot_controller, robot_twist
    global fatigue_threshold, radius_threshold
    global ball_detected, ball_is_close
    global time_since, last_detection, detection_time, time_before_change_target
    global max_speed
    global yaw, robot_pose

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
    #   \brief Definition of the controller which directly controls the robot velocity
    robot_controller = None

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
