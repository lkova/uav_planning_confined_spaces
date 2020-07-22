#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer.lookupTransform("ar_marker_2", "tank", ros::Time(0));
      std::cout << transformStamped.transform.translation.x << " " << transformStamped.transform.translation.y << " "
                << transformStamped.transform.translation.z << std::endl;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
  return 0;
}
