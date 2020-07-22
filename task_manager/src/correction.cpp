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
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <time.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>

bool isDetected = false;
geometry_msgs::Point pp;
geometry_msgs::Pose pose;
int counter = 0;
Eigen::Vector3d vec;
Eigen::Vector3d desiredPosition;
Eigen::Vector3d error;

double cx = 0.0, cy = 0.0, cz = 2.0;

void currentPose(const geometry_msgs::PosePtr &msg)
{
  desiredPosition[0] = msg->position.x;
  desiredPosition[1] = msg->position.y;
  desiredPosition[2] = msg->position.z;
}

void callback(const geometry_msgs::TransformStampedPtr &msg)
{
  error[1] = msg->transform.translation.x;
  error[2] = msg->transform.translation.y;
  error[0] = -msg->transform.translation.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "correction");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 10);

  ros::Subscriber subscriber = nh_private.subscribe("/visual_servoing/error_transform", 1, callback);
  ros::Subscriber subscriber_2 = nh_private.subscribe("/firefly/odometry_sensor1/pose", 1, currentPose);
  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < 10; i++)
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  desiredPosition -= error;
  std::cout << desiredPosition << std::endl;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desiredPosition, 0.0, &trajectory_msg);

  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();
  ros::Duration(5.0).sleep();
  ros::shutdown();

  return 0;
}
