#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <adaptive_planning/planner_srv.h>
#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <hole_detection/Hole.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

Eigen::Vector3d drone_position;
ros::Publisher trajectory_pub;
double msg_yaw;

void callbackNew(const hole_detection::Hole msg) {
  std::cout << "hole type " << msg.type << std::endl;
  std::cout << "hole point " << msg.point.x << " " << msg.point.y << " "
            << msg.point.z << std::endl;
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
    Eigen::Vector3d front_hole;
    front_hole[0] = p2.point.x;
    front_hole[1] = p2.point.y;
    front_hole[2] = p2.point.z;
    ROS_INFO("doslo FRONT %lf %lf %lf", front_hole.x(), front_hole.y(),
             front_hole.z());
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
}

void deleteThis() {
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  Eigen::Vector3d desired_position(5.5, -0.75, 1.2);

  // Eigen::Vector3d desired_position(5.2, -0.8, 1.3);

  // trajectory_msg.header.stamp = ros::Time::now();

  // desired_position[0] = 6.5;
  // desired_position[1] = 0;
  // desired_position[2] = 10;
  // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0.0,
  // &trajectory_msg); trajectory_pub.publish(trajectory_msg); ros::spinOnce();
  // ros::Duration(2.0).sleep();

  // trajectory_msg.header.stamp = ros::Time::now();
  // desired_position[0] = 6.5;
  // desired_position[1] = 0;
  // desired_position[2] = 6;
  // mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0.0,
  // &trajectory_msg); trajectory_pub.publish(trajectory_msg); ros::spinOnce();
  // ros::Duration(2.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();

  // desired_position[0] = 5.5;
  // desired_position[1] = -0.75;
  // desired_position[2] = 1.2;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0.0,
                                                      &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);
  ros::spinOnce();
  ros::Duration(2.0).sleep();
}
void poseCallback(const geometry_msgs::PoseConstPtr &msg) {
  drone_position[0] = msg->position.x;
  drone_position[1] = msg->position.y;
  drone_position[2] = msg->position.z;
  tf::Pose pose;
  tf::poseMsgToTF(*msg, pose);
  msg_yaw = tf::getYaw(pose.getRotation());
  /*msg_yaw = tf::getYaw(pose.getRotation()) + M_PI;
  if (msg_yaw > M_PI)
  {
    msg_yaw -= 2 * M_PI;
  }*/
  // ROS_INFO("real yaw %lf", msg_yaw);
}

void turn(double desired_yaw, Eigen::Vector3d desired_position) {
  double dist = atan2(sin(desired_yaw - msg_yaw), cos(desired_yaw - msg_yaw));
  double sgn = dist / fabs(dist);
  // std::cout << "distr " << sgn << std::endl;
  // std::cout << "diff " << dist << std::endl;
  dist = fabs(dist);
  double go_to_angle = msg_yaw;
  while (dist > 0.7) {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    dist -= 0.7;
    go_to_angle += 0.7 * sgn;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, go_to_angle, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    // ROS_INFO("you did this cur angle %lf", go_to_angle);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();
  ros::Duration(1.0).sleep();

  // ROS_INFO("done %lf %lf", msg_yaw, desired_yaw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "firefly/command/trajectory", 10);
  // ros::Subscriber sub_front = nh.subscribe<geometry_msgs::PointStamped>(
  //     "/firefly/detectionFront/pointy", 1, callbackFront);
  ros::Subscriber sub_front =
      nh.subscribe("/firefly/detectionFront/hole", 1, callbackNew);
  ROS_INFO("Started hovering example.");

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>(
      "/firefly/odometry_sensor1/pose", 1, poseCallback);

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  int n_seq = 0;
  ros::spinOnce();
  ros::Duration(2.0).sleep();

  deleteThis();

  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    adaptive_planning::planner_srv plannerSrv;
    plannerSrv.request.goal_set = false;

    if (ros::service::call("planner", plannerSrv)) {
      ROS_INFO("planner srv size %d", plannerSrv.response.path.size());

      if (plannerSrv.response.success) {
        if (plannerSrv.response.path.size() > 0) {
          for (int i = 0; i < plannerSrv.response.path.size(); i++) {
            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();

            Eigen::Vector3d desired_position(
                plannerSrv.response.path[i].position.x,
                plannerSrv.response.path[i].position.y,
                plannerSrv.response.path[i].position.z);
            tf::Pose pose;
            tf::poseMsgToTF(plannerSrv.response.path[i], pose);

            double ddx =
                plannerSrv.response.path[i].position.x - drone_position.x();
            double ddy =
                plannerSrv.response.path[i].position.y - drone_position.y();

            double first_yaw = atan2(ddy, ddx);

            // std::cout << "final dest " << desired_position << std::endl;
            // std::cout << "yaw " << first_yaw << std::endl;
            // first turn

            turn(first_yaw, drone_position);
            // go

            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
                desired_position, first_yaw, &trajectory_msg);
            trajectory_pub.publish(trajectory_msg);
            ros::spinOnce();
            ros::Duration(1.0).sleep();

            // second turn
            double yaw = tf::getYaw(pose.getRotation());

            // first turn
            turn(yaw, desired_position);
            ros::spinOnce();
            ros::Duration(2.0).sleep();
          }
        }
      }
    } else {
      ROS_INFO("planning failed");
    }
  }
  return 0;
}
