from geometry_msgs.msg import Pose


robot_pose = Pose()

##
#    \brief odometryReceived stores the value of the current robot heading
#    \param msg is the robot current odometry
#
#    This function simply process the received odometry message in order to
#    to obtain the current heading of the robot.
#
def odometryReceived(msg):
    global robot_pose
    robot_pose = msg.pose.pose
