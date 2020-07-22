#include <ros/ros.h>
#include "hole_detection/HoleDetectionPackage.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_template");
  ros::NodeHandle nodeHandle("~");

  hole_detection::HoleDetectionPackage HoleDetectionPackage(nodeHandle);
  ros::spin();
  return 0;
}