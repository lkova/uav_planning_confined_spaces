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

bool isready_tank1 = false;
bool isready_tank2 = false;
bool found_world_t1 = true;
bool found_world_t2 = true;

ros::Publisher pub;
// parameters
std::string camera_topic = "/mynteye_left_mono_frame";
std::unordered_map<int, geometry_msgs::Pose> ar_tags_tank;
std::string ground_truth_frame = "ground_truth";
std::string frame_id_tank1 = "tank1";
std::string path_tank1 = "/home/luka/uav2_ws/src/adaptive_path_planning_for_uav/task_manager/resource/artags.txt";
std::string frame_id_tank2 = "tank2";
std::string path_tank2 = "/home/luka/uav2_ws/src/adaptive_path_planning_for_uav/task_manager/resource/artags2.txt";

float fx = 763.0;
float fy = 763.0;
float cx = 618.0;
float cy = 399.0;
float width = 1280;
float height = 720;
/*
    Load tags from tag file. Tags are also published as a node, but this was easier to do
*/

void publishGroundTruthPose(geometry_msgs::TransformStamped &transform)
{
  static tf::TransformListener listener_t1;
  tf::StampedTransform transform_t1;

  geometry_msgs::PoseStamped pose_t, pose_pub;

  pose_t.header = transform.header;
  pose_t.pose.position.x = transform.transform.translation.x;
  pose_t.pose.position.y = transform.transform.translation.y;
  pose_t.pose.position.z = transform.transform.translation.z;
  pose_t.pose.orientation.x = transform.transform.rotation.x;
  pose_t.pose.orientation.y = transform.transform.rotation.y;
  pose_t.pose.orientation.z = transform.transform.rotation.z;
  pose_t.pose.orientation.w = transform.transform.rotation.w;

  try
  {
    listener_t1.lookupTransform("world", frame_id_tank1, ros::Time(0), transform_t1);
    listener_t1.transformPose("world", pose_t, pose_pub);
    pub.publish(pose_pub);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

bool isTagGood(Eigen::Matrix4f trans, float &distance)
{
  Eigen::Vector3f position = trans.block<3, 1>(0, 3);
  float u, v;
  u = position.x() * fx / position.z() + cx;
  v = position.y() * fy / position.z() + cy;

  float delta = height * 0.1;
  float scaling_factor = 1.0;

  if (position.z() > 2.0)
    scaling_factor *= fabs(position.z());

  distance = sqrt(pow(u - cx, 2) + pow(v - cy, 2));
  if (u < delta || u > (width - delta) || v < delta || v > (height - delta))
  {
    std::cout << "IT IS FALSE u, v " << u << " " << v << std::endl;
    return false;
  }

  return true;
}
void loadTags(std::string path_to_file, bool &isready)
{
  std::ifstream myfile(path_to_file);

  int i = 0;
  float unit = 100.0;
  if (myfile.is_open())
  {
    int id;
    float x, y, z, r, p, yaw;
    while (myfile >> id >> x >> y >> z >> r >> p >> yaw)
    {
      geometry_msgs::Pose pose;

      pose.position.x = x / unit;
      pose.position.y = y / unit;
      pose.position.z = z / unit;
      tf2::Quaternion quat;
      quat.setRPY(r, p, yaw);
      pose.orientation.x = quat.x();
      pose.orientation.y = quat.y();
      pose.orientation.z = quat.z();
      pose.orientation.w = quat.w();
      ar_tags_tank[id] = pose;
      isready = true;
    }
    myfile.close();
  }

  else
    std::cout << "Unable to open file" << std::endl;
}

void callback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &msg)
{
  // if the tags are loaded

  if (isready_tank1 && isready_tank2)
  {
    float dist_trans_minimum = 10000.0;

    geometry_msgs::TransformStamped best_transform_stamped;

    for (int i = 0; i < msg->markers.size(); i++)
    {
      tf::StampedTransform transform2;
      static tf::TransformListener listener2;
      bool is_transform_ready = true;

      try
      {
        // Find camera in the ar marker frame
        std::string marker_id = "ar_marker_" + std::to_string(msg->markers[i].id);
        listener2.lookupTransform(marker_id, camera_topic, ros::Time(0), transform2);
        is_transform_ready = false;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      if (is_transform_ready)
        continue;
      int id = msg->markers[i].id;

      if (ar_tags_tank.find(id) != ar_tags_tank.end())
      {
        // transformations between ar_markers_frame to ar_tags_frame
        // fixed transformation
        Eigen::Quaternionf quat_marker_tag(-0.5, 0.5, 0.5, 0.5);

        Eigen::Matrix4f trans_marker_tag;
        trans_marker_tag.setIdentity();
        trans_marker_tag.block<3, 3>(0, 0) = quat_marker_tag.toRotationMatrix();

        // transformation camera in the ar marker frame
        Eigen::Quaternionf quat_marker_cam;
        quat_marker_cam.x() = transform2.getRotation().x();
        quat_marker_cam.y() = transform2.getRotation().y();
        quat_marker_cam.z() = transform2.getRotation().z();
        quat_marker_cam.w() = transform2.getRotation().w();
        Eigen::Matrix3f R_marker_cam = quat_marker_cam.toRotationMatrix();
        Eigen::Vector3f t_marker_cam;
        t_marker_cam.x() = transform2.getOrigin().x();
        t_marker_cam.y() = transform2.getOrigin().y();
        t_marker_cam.z() = transform2.getOrigin().z();

        Eigen::Matrix4f trans_marker_cam;
        trans_marker_cam.setIdentity();
        trans_marker_cam.block<3, 3>(0, 0) = quat_marker_cam.toRotationMatrix();
        trans_marker_cam.block<3, 1>(0, 3) = t_marker_cam;

        // CHECK IF IT IS IN THE GOOD POSITION
        float distance_to_center = 10000.0;
        if (!isTagGood(trans_marker_cam.inverse(), distance_to_center))
        {
          continue;
        }
        std::cout << "good id " << id << std::endl;
        // ar marker to markerFrame
        Eigen::Vector3f ar_vec;
        ar_vec.x() = ar_tags_tank[id].position.x;
        ar_vec.y() = ar_tags_tank[id].position.y;
        ar_vec.z() = ar_tags_tank[id].position.z;

        Eigen::Quaternionf quat_ar;
        quat_ar.x() = ar_tags_tank[id].orientation.x;
        quat_ar.y() = ar_tags_tank[id].orientation.y;
        quat_ar.z() = ar_tags_tank[id].orientation.z;
        quat_ar.w() = ar_tags_tank[id].orientation.w;

        Eigen::Matrix4f trans_final;
        trans_final.setIdentity();
        trans_final.block<3, 3>(0, 0) = quat_ar.toRotationMatrix();
        trans_final.block<3, 1>(0, 3) = ar_vec;

        Eigen::Matrix4f result = trans_marker_tag.inverse() * trans_marker_cam;
        // std::cout << "camera in tag frame " << result.block<3, 1>(0, 3) << std::endl;
        // std::cout << std::endl;
        std::cout << "camera in marker frame" << std::endl;
        Eigen::Matrix4f trans_target_cam = (trans_final * result);
        std::cout << (trans_final * result).block<3, 1>(0, 3) << std::endl;

        // static tf2_ros::TransformBroadcaster broadcaster;

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();

        std::string tank_id;
        if (id < 11)
        {
          tank_id = frame_id_tank1;
        }
        else
        {
          tank_id = frame_id_tank2;
        }
        transformStamped.header.frame_id = tank_id;
        std::string tag_string = ground_truth_frame;
        transformStamped.child_frame_id = tag_string;
        Eigen::Vector3f position_fin = trans_target_cam.block<3, 1>(0, 3);
        Eigen::Quaternionf quat_fin(trans_target_cam.block<3, 3>(0, 0));

        transformStamped.transform.translation.x = position_fin.x();
        transformStamped.transform.translation.y = position_fin.y();
        transformStamped.transform.translation.z = position_fin.z();

        transformStamped.transform.rotation.x = quat_fin.x();
        transformStamped.transform.rotation.y = quat_fin.y();
        transformStamped.transform.rotation.z = quat_fin.z();
        transformStamped.transform.rotation.w = quat_fin.w();
        // broadcaster.sendTransform(transformStamped);

        if (distance_to_center < dist_trans_minimum)
        {
          dist_trans_minimum = distance_to_center;
          best_transform_stamped = transformStamped;
        }
      }
    }
    static tf2_ros::TransformBroadcaster broadcaster;
    if (dist_trans_minimum != 10000.0)
    {
      broadcaster.sendTransform(best_transform_stamped);
      std::cout << "are you sending it " << std::endl;
      publishGroundTruthPose(best_transform_stamped);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alvar");

  ros::NodeHandle node;
  //   ros::Rate rate = 60;

  loadTags(path_tank1, isready_tank1);
  loadTags(path_tank2, isready_tank2);

  ros::Subscriber sub = node.subscribe("/ar_pose_marker", 1, callback);
  pub = node.advertise<geometry_msgs::PoseStamped>("ground_truth_pose", 1);
  ros::spin();
  return 0;
}
