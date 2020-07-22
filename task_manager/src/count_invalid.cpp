#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

double x;
double y;
double z;
int down_found = 0;
int front_found = 0;
double px;
double q;
double r;
int good = 0;
int bad = 0;
void callbackDown(const geometry_msgs::PointStampedConstPtr msg) {
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    geometry_msgs::PointStamped p_world;
    geometry_msgs::PointStamped p2;
    listener.lookupTransform(
        "world", "firefly/vi_sensor_down/camera_depth_optical_center_link",
        ros::Time(0), transform);
    listener.transformPoint("world", *msg, p2);
    x = p2.point.x;
    y = p2.point.y;
    z = p2.point.z;
    // ROS_INFO("doslo down %lf %lf %lf", x, y, z);
    if (x > 6.40 && x < 6.8 && y > -0.15 && y < 0.15 && z > 6.5 && z < 7.5) {
      good++;
    } else {
      bad++;
    }
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
}

void callbackFront(const geometry_msgs::PointStampedConstPtr msg) {
  static tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    geometry_msgs::PointStamped p_world;
    geometry_msgs::PointStamped p2;
    listener.lookupTransform(
        "world", "firefly/vi_sensor/camera_depth_optical_center_link",
        ros::Time(0), transform);
    listener.transformPoint("world", *msg, p2);
    x = p2.point.x;
    y = p2.point.y;
    z = p2.point.z;
    front_found = 1;
    ROS_INFO("doslo FRONT %lf %lf %lf", x, y, z);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
}

void poseCallback(const geometry_msgs::PoseConstPtr &msg) {
  px = msg->position.x;
  q = msg->position.y;
  r = msg->position.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Subscriber sub_down = nh.subscribe<geometry_msgs::PointStamped>(
      "/firefly/detectionDown/hole_central_point", 1, callbackDown);
  ros::Subscriber sub_front = nh.subscribe<geometry_msgs::PointStamped>(
      "/firefly/detectionFront/hole_central_point", 1, callbackFront);

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "firefly/command/trajectory", 10);
  ROS_INFO("Started hovering example.");

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>(
      "/firefly/vi_sensor/ground_truth/pose", 1, poseCallback);

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  int n_seq = 0;
  ros::Duration(1.0).sleep();

  while (ros::ok()) {
    ROS_INFO("good %d , bad %d", good, bad);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  // ROS_INFO("good %d , bad %d", good, bad);
  // ros::spin();
  // ros::Duration(1.0).sleep();
  return 0;
}
