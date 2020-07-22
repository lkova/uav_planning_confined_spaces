#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <unordered_map>

std::string target1 = "world";
std::string target2 = "/abs_correction";
std::string camera_topic = "/mynteye_left_mono_frame";
ros::Publisher pub_abs, pub_w, pub_ar_abs, pub_ar_w;

void callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
  for (int i = 0; i < msg->markers.size(); i++)
  {
    tf::StampedTransform transform_w, transform_abs;
    static tf::TransformListener listener_w, listener_abs;
    geometry_msgs::PoseStamped pose_world, pose_abs;
    ar_track_alvar_msgs::AlvarMarker m_w, m_abs;

    try
    {
      listener_w.lookupTransform(target1, camera_topic, ros::Time(0), transform_w);
      geometry_msgs::PoseStamped pose_temp;
      pose_temp = msg->markers[i].pose;
      pose_temp.header.frame_id = camera_topic;
      listener_w.transformPose(target1, pose_temp, pose_world);
      m_w = msg->markers[i];
      m_w.pose.header.frame_id = target1;
      m_w.header.frame_id = target1;
      m_w.pose = pose_world;
      pub_w.publish(pose_world);
      pub_ar_w.publish(m_w);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    try
    {
      listener_abs.lookupTransform(target2, camera_topic, ros::Time(0), transform_abs);
      geometry_msgs::PoseStamped pose_temp;
      pose_temp = msg->markers[i].pose;
      pose_temp.header.frame_id = camera_topic;
      listener_abs.transformPose(target2, pose_temp, pose_abs);
      m_abs = msg->markers[i];
      m_abs.pose = pose_abs;
      m_abs.header.frame_id = target2;
      m_abs.pose.header.frame_id = target2;
      m_abs.pose = pose_abs;
      pub_abs.publish(pose_abs);
      pub_ar_abs.publish(m_abs);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "abs");

  ros::NodeHandle node;
  //   ros::Rate rate = 60;

  ros::Subscriber sub = node.subscribe("/ar_pose_marker", 1, callback);
  pub_abs = node.advertise<geometry_msgs::PoseStamped>("marker_abs", 1);
  pub_w = node.advertise<geometry_msgs::PoseStamped>("marker_world", 1);
  pub_ar_abs = node.advertise<ar_track_alvar_msgs::AlvarMarker>("marker_ar_abs", 1);
  pub_ar_w = node.advertise<ar_track_alvar_msgs::AlvarMarker>("marker_ar_world", 1);
  ros::spin();
  return 0;
}