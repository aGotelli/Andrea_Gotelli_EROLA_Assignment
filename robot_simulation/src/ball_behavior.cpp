#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_simulation_messages/PlanningAction.h>









int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_behavior");
  ros::NodeHandle nh_glob;
  //  Declare the action server client
  actionlib::SimpleActionClient<robot_simulation_messages::PlanningAction> plan_ball_motion("reaching_goal", true);
  //  Wait for the service to be available
  plan_ball_motion.waitForServer();

  //  Define the node frame rate
  ros::Rate node_frame_rate(50);
  while(ros::ok()) {

  }
  ROS_INFO("Hello world!");
}
