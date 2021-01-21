import math
import random
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from robot_simulation_state_machines.reach_goal import reachPosition
from robot_simulation_state_machines.reach_goal import planning_client
import robot_simulation_state_machines.image_processing as imp
import robot_simulation_state_machines.robot_position as rp


##
#   \brief Define the width of the arena.
width = 0

##
#   \brief Define the height of the arena.
height = 0

##
#   \brief Defines the maximum level of fatigue the robot can andle before going to sleep.
fatigue_threshold = 0

time_before_change_target = 0.0


##
#    \brief isTired check the level of fatigue to establish if the robot is "tired"
#    \param level [integer] is the current level of fatigue of the robot.
#    \return a boolen. True if the robot is "tired" false elsewhere.
#
#    This function compares the level of fatigue with and hard threshold which is a parameter
#    eccessible for the user.
#
def isTired(fatigue_level):
    if fatigue_level >= fatigue_threshold:
        return True
    else :
        return False



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
                                 outcomes=['tired','tracking', 'play'],
                                 input_keys=['move_fatigue_counter_in'],
                                 output_keys=['move_fatigue_counter_out','move_room_to_find'])
            self.target = Pose()
            self.targetSeemsReacheable = True
            self.prev_dist = 0.0
            self.time_to_play = False
            self.sub = rospy.Subscriber("person_willing",String, self.commandReceived,  queue_size=1)


    def commandReceived(self, msg):
        print("Received command to play")
        self.time_to_play = True

    def newTarget(self, target=None):
        self.targetSeemsReacheable = True
        #   Declare a geometry_msgs/Pose for the random position
        random_ = Pose()
        #   Define the random components (x, y) of this random position
        if (target == None):
            random_.position.x = random.randint(-width/2, width/2)
            random_.position.y = random.randint(-height/2, height/2)
        else:
            print("Using user-defined coordinates")
            random_.position.x = target[0]
            random_.position.y = target[1]
        #   Call the service to reach this position
        self.target = random_
        self.prev_dist = compEuclidDist(random_, rp.robot_pose)

        reachPosition(random_, verbose=True)




    def checkReachability(self, event):
        curr_dist = compEuclidDist(self.target, rp.robot_pose)
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
        userdata.move_room_to_find = ''
        #   Declare a geometry_msgs/Pose for the random position
        self.newTarget()
        #   Initialize the timer to check if the robot progress periodically
        timer = rospy.Timer(rospy.Duration(time_before_change_target), self.checkReachability)
        #   Main loop
        while not rospy.is_shutdown():
            if self.time_to_play:
                timer.shutdown()
                return 'play'
            #   Check if the person has commanded play
            if imp.ball_detected :
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
                state = planning_client.get_state()
                if state == 3:
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
