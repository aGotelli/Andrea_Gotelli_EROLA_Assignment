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
const inline ros::Duration waitingPeriod(const int min=300, const int max=600)
{
  return ros::Duration( rand()%(max-min + 1) + min );
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
void CallToPlay(ros::TimerEvent, ros::Timer* timer)
{
  //  Create an publish an empty message
  command_to_play.publish( std_msgs::String() );

  //  Reset the timer
  timer->stop();
  timer->setPeriod( waitingPeriod() );
  timer->start();
}

/*!
 * \brief roomSelection selects a room radomly in the list when requested
 * \param res is the response of the service callback containing the room name
 * \return true, as this service cannot fail.
 *
 * This function is executed when the service is called. In its execusion, this function
 * randomly mexes the element in the vector rooms using std::random_shuffle. Then it picks
 * the first element of the randomly sorted vector.
 */
bool roomSelection(robot_simulation_messages::PersonCommand::Request  &,
         robot_simulation_messages::PersonCommand::Response &res)
{
  std::random_shuffle(rooms.begin(), rooms.end());
  res.room = rooms.front();
  return true;
}

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

  //  Declare and initialize the timer
  ros::Timer auto_caller;
  auto_caller = nh.createTimer( waitingPeriod(), boost::bind(CallToPlay, _1, &auto_caller), true );

  //  Main loop
  ros::spin();

  return 0;

}
