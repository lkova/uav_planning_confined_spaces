#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include "geometry_msgs/PointStamped.h"

geometry_msgs::PointStamped new_p;

void callbackClear(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  Eigen::Vector3d down_hole;
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    geometry_msgs::PointStamped p_world;
    geometry_msgs::PointStamped p2;
    listener.lookupTransform("world", "firefly/vi_sensor/camera_depth_optical_center_link", ros::Time(0), transform);
    listener.transformPoint("world", *msg, p2);
    down_hole[0] = p2.point.x;
    down_hole[1] = p2.point.y;
    down_hole[2] = p2.point.z;
    ROS_INFO("doslo down %lf %lf %lf", down_hole.x(), down_hole.y(), down_hole.z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform");

  ros::NodeHandle node("~");
  // ros::Subscriber sub = node.subscribe("/pelican/odometry_sensor1/position", 1, callbackClear);
  ros::Subscriber sub = node.subscribe("/firefly/detectionFront/pointy", 1, callbackClear);
  ros::Publisher pub = node.advertise<geometry_msgs::PointStamped>("point_world", 1);

  ros::spin();

  return 0;
}