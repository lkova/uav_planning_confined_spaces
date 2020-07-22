#include <ros/ros.h>
#include "visual_servoing/VisualServoingPackage.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servoing_package");
  ros::NodeHandle nodeHandle("~");

  visual_servoing::VisualServoingPackage visual_servoing_package(nodeHandle);
  ros::spin();
  return 0;
}