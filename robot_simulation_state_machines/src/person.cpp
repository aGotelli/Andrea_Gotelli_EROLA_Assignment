#include "ros/ros.h"
#include <std_msgs/String.h>
#include <robot_simulation_messages/PersonCommand.h>
#include <algorithm>
#include <ctime>
#include <cstdlib>

static std::vector<std::string> rooms {
  "Entrance",
  "Closet",
  "LivingRoom",
  "Kitchen",
  "Bathroom",
  "Bedroom"
};


const inline ros::Duration waitingPeriod(const int min=300, const int max=600)
{
  return ros::Duration( rand()%(max-min + 1) + min );
}


static ros::Publisher command_to_play;
void CallToPlay(ros::TimerEvent, ros::Timer* timer)
{
  command_to_play.publish( std_msgs::String() );
  timer->stop();
  timer->setPeriod( waitingPeriod() );
  timer->start();
}


bool roomSelection(robot_simulation_messages::PersonCommand::Request  &,
         robot_simulation_messages::PersonCommand::Response &res)
{
  std::random_shuffle(rooms.begin(), rooms.end());
  res.room = rooms.front();
  return true;
}

int main(int argc, char **argv)
{
  std::srand ( unsigned ( std::time( nullptr) ) );

  ros::init(argc, argv, "person");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/person_decision", roomSelection);

  command_to_play = nh.advertise<std_msgs::String>("PersonCommand", 1);

  ros::Timer auto_caller;
  auto_caller = nh.createTimer( waitingPeriod(), boost::bind(CallToPlay, _1, &auto_caller), true );

  ros::spin();

  return 0;

}
