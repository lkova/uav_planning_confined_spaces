/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <thread>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <time.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>

bool isDetected = false;
geometry_msgs::Point pp;
int counter = 0;
Eigen::Vector3d vec;
double cx = 0.0, cy = 0.0, cz = 2.0;

void path(const geometry_msgs::PointConstPtr &msg)
{
  pp.x = msg->x;
  pp.y = msg->y;
  pp.z = msg->z;
  if (pp.x > 0.0 && pp.y > 0.0 && pp.z > 2.0)
  {
    cx = pp.x;
    cy = pp.y;
    cz = pp.z;
  }
  else
  {
    cx = 0.0;
    cy = 0.0;
    cz = 2.0;
  }
  // ROS_INFO("IN HOLE %lf %lf %lf", pp.x, pp.y, pp.z);
}

void callback(const geometry_msgs::PointStampedPtr &msg)
{
  if (!isDetected)
  {
    isDetected = true;
    pp.x = msg->point.x;
    pp.y = msg->point.y;
    pp.z = msg->point.z;
    ROS_INFO("tocka %lf %lf %lf ", pp.x, pp.y, pp.z);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else
  {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  ros::Subscriber subscriber = nh_private.subscribe("/firefly/transform/point_world", 1, callback);
  ros::Subscriber subscriber_path = nh_private.subscribe("/firefly/zpoint", 1, path);
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  // Eigen::Vector3d desired_position(5, 0.5, 3.5);
  Eigen::Vector3d desired_position(2.15, -1.3, 1.5);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

  trajectory_pub.publish(trajectory_msg);

  ros::Rate rate(1.0);
  ros::spinOnce();
  ros::Duration(5.0).sleep();
  /*
    while (nh.ok())
    {
      Eigen::Vector3d desired(cx, cy, cz);
      // z = z + 1.0;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired, desired_yaw, &trajectory_msg);

      trajectory_pub.publish(trajectory_msg);
      ros::spinOnce();
      rate.sleep();
    }

    double z = 3.0;
    while (z < 6.0)
    {
      Eigen::Vector3d desired(0, 0.0, z);
      z = z + 1.0;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired, desired_yaw, &trajectory_msg);

      trajectory_pub.publish(trajectory_msg);
      ROS_INFO("IN HOLE %lf %lf %lf", pp.x, pp.y, pp.z);
      ros::spinOnce();
      rate.sleep();
    }*/
  /*
    Eigen::Vector3d desired_position2(4.0, 0.0, 10.5);
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position2, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    ros::spinOnce();
    rate.sleep();*/
  /*
    double x = 4.0;
    while (x < 13.1)
    {
      Eigen::Vector3d desired(x, 0.0, 10.5);
      x = x + 1.0;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired, desired_yaw, &trajectory_msg);

      trajectory_pub.publish(trajectory_msg);
      ros::spinOnce();
      rate.sleep();
    }*/
  /*
  ros::Duration(10.0).sleep();
  Eigen::Vector3d desired_position2(5.0, 0.0, 11.5);
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position2, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  ros::Rate rate(10.0);

  while (nh.ok())
  {
    if (isDetected)
    {
      Eigen::Vector3d desired_position3(3.8 + pp.x, pp.y, 4.0 + pp.z);
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position3, desired_yaw, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }
  */
  ros::shutdown();

  return 0;
}
