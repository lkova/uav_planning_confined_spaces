#include <ros/ros.h>
#include <stdlib.h>
#include "adaptive_planning/Planner.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nodeHandle;

  planner::Planner planner(nodeHandle);

  ros::spin();
  return 0;
}