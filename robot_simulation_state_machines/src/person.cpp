#include "ros/ros.h"
#include <robot_simulation_messages/PersonCommand.h>
#include <algorithm>
#include <ctime>
#include <cstdlib>

static std::vector<std::string> rooms {
  "entrance",
  "closet",
  "living_room",
  "kitchen",
  "bathroom",
  "bedroom"
};

bool room_selection(robot_simulation_messages::PersonCommand::Request  &req,
         robot_simulation_messages::PersonCommand::Response &res)
{
  std::random_shuffle(rooms.begin(), rooms.end());
  res.room = rooms.front();
  return true;
}

int main(int argc, char **argv)
{
  std::srand ( unsigned ( std::time(0) ) );

  ros::init(argc, argv, "person");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/person_decision", room_selection);

  ros::spin();

  return 0;
}
