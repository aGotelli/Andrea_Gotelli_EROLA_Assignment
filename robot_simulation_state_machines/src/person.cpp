/**
 * \file person.cpp
 * \brief This files emulates the a person giving commands to a robot
 * \author Andrea Gotelli
 * \version 0.1
 * \date 27/01/2021
 *
 *
 * \details
 *
 * Subscribes to: <BR>
 *    ° [None]
 *
 * Publishes to: <BR>
 *    ° /PersonCommand
 *
 * Service : <BR>
 *    ° /person_decision
 *
 * Description :
 *
 * This node simulates a person behavior. The person is assumed to want to play with the robot at some
 * time randomly. The person calls the robot to play in an interval which is in the range [600, 900] seconds
 * corresponding to 10-15 minutes. When it decides to play with the robot, it publishes the command in the
 * topic: /PersonCommand.
 * It also simulates a person taking time to chose the room. This is done in the service callback,
 * which waits for some time before giving the random answer.
 *
 */


#include "ros/ros.h"
#include <std_msgs/String.h>
#include <robot_simulation_messages/PersonCommand.h>
#include <algorithm>
#include <ctime>
#include <cstdlib>

/// \brief rooms is declarared as s static vector of strings, represents the list of rooms
static std::vector<std::string> rooms {
  "Entrance",
  "Closet",
  "LivingRoom",
  "Kitchen",
  "Bathroom",
  "Bedroom"
};


/*!
 * \brief waitingPeriod returns a ros::Duration element of a random number of seconds
 * \param min defines the lower bound of the range [default 300 s]
 * \param max defines the upper bound of the range [default 600 s]
 * \return the constant ros::Duration
 */
const inline ros::Duration waitingPeriod(const int min=300, const int max=400, const bool verbose=true)
{
  const int time_to_next_call = rand()%(max-min + 1) + min;
  if(verbose)
    ROS_INFO_STREAM("Next call to play will arrive in : " << time_to_next_call << " [s]" );
  return ros::Duration(time_to_next_call);
}


/// \brief command_to_play is the ros publisher for commanding the person intention.
static ros::Publisher command_to_play;

/*!
 * \brief CallToPlay  is the timer callback to send the message with the person intention.
 * \param timer pointer to the calling timer to reset the rate. 
 * 
 * This function simply generates an empty message in order to emulate the person to command its intention
 * to play or interact with the robot. As this command should be followed by another one in the future, the rate of the
 * timer is reset at each call of the function.
 */
void CallToPlay(ros::TimerEvent)
{
  //  Create an publish an empty message
  command_to_play.publish( std_msgs::String() );
}

/// \brief auto_caller is a ros::Timer which executes the function calling the robot at a given frequency
static ros::Timer auto_caller;

/*!
 * \brief resetTimer stops the timer and reinitialise its rate.
 */
void resetTimer()
{
  auto_caller.stop();
  auto_caller.setPeriod( waitingPeriod() );
  auto_caller.start();
}

/*!
 * \brief roomSelection selects a room radomly in the list when requested
 * \param res is the response of the service callback containing the room name
 * \return true, as this service cannot fail.
 *
 * This function is executed when the service is called. In its execusion, first checks the robot intention.
 * If the robot wants to play, it randomly mixes the elements in the vector rooms using std::random_shuffle.
 * Then it picks the first element of the randomly sorted vector. In the second case, if the robot does not
 * want to play anynore (information contained in the request: want_to_play) it calls resetTimer().
 * This function returns true in both cases, as this service cannot fail.
 */
bool roomSelection(robot_simulation_messages::PersonCommand::Request& request,
         robot_simulation_messages::PersonCommand::Response& res)
{
  //  Check if the robot has sent the command of playing
  if( request.want_play) {
    std::random_shuffle(rooms.begin(), rooms.end());
    res.room = rooms.front();
    auto decision_time = waitingPeriod(2,4, false);
    decision_time.sleep();
    return true;
  }
  //  Else the call was to tell the robot stops playing
  //  Reset timer for the next call to play
  resetTimer();
  return true;
}

/*!
 * \brief main initialises the node and interfaces.
 *
 * This function initialises the node, the node handle, the publisher and the service provider needed for the simulation of the
 * person behavior.
 */
int main(int argc, char **argv)
{
  //  Init randomness
  std::srand ( unsigned ( std::time( nullptr) ) );

  //  Init node
  ros::init(argc, argv, "person");

  //  Node handle
  ros::NodeHandle nh;

  //  Declare service provider
  ros::ServiceServer service = nh.advertiseService("/person_decision", roomSelection);

  //  Initialize publisher
  command_to_play = nh.advertise<std_msgs::String>("PersonCommand", 1);

  //  Wait initialization to stop before starting
  ros::Duration waiting_time(1);
  waiting_time.sleep();

  //  Initialize the timer (first call not too far in the future)
  auto_caller = nh.createTimer( waitingPeriod(20, 30), CallToPlay, true );

  //  Main loop
  ros::spin();

  return 0;

}
