#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>

std::string frame_id_tank1 = "tank1";
std::string path_tank1 = "/home/luka/uav2_ws/src/adaptive_path_planning_for_uav/task_manager/resource/artags.txt";
std::string frame_id_tank2 = "tank2";
std::string path_tank2 = "/home/luka/uav2_ws/src/adaptive_path_planning_for_uav/task_manager/resource/artags2.txt";

void loadTags(std::map<int, geometry_msgs::TransformStamped> &ar_tags, std::string frame_id, std::string path_to_file)
{
  std::ifstream myfile(path_to_file);

  float unit = 100.0;
  if (myfile.is_open())
  {
    int id;
    float x, y, z, r, p, yaw;
    while (myfile >> id >> x >> y >> z >> r >> p >> yaw)
    {
      geometry_msgs::TransformStamped static_transformStamped;
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = frame_id;
      std::string tag_string = "ar_tag_" + std::to_string(id);
      static_transformStamped.child_frame_id = tag_string;
      static_transformStamped.transform.translation.x = x / unit;
      static_transformStamped.transform.translation.y = y / unit;
      static_transformStamped.transform.translation.z = z / unit;
      tf2::Quaternion quat;
      quat.setRPY(r, p, yaw);
      static_transformStamped.transform.rotation.x = quat.x();
      static_transformStamped.transform.rotation.y = quat.y();
      static_transformStamped.transform.rotation.z = quat.z();
      static_transformStamped.transform.rotation.w = quat.w();
      ar_tags[id] = static_transformStamped;
    }
    myfile.close();
  }

  else
    std::cout << "Unable to open file" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::map<int, geometry_msgs::TransformStamped> ar_tags;

  loadTags(ar_tags, frame_id_tank1, path_tank1);
  loadTags(ar_tags, frame_id_tank2, path_tank2);

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  //   static tf2_ros::TransformBroadcaster static_broadcaster;
  for (const auto &pair : ar_tags)
  {
    static_broadcaster.sendTransform(pair.second);
  }
  ros::spin();
  return 0;
}
