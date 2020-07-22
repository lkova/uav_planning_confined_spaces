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

#include <adaptive_planning/planner_srv.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <cmath>

bool isDetected = false;
geometry_msgs::Point pp;
geometry_msgs::PoseStamped pose;
int counter = 0;
Eigen::Vector3d desiredPosition, error;

void currentPose(const geometry_msgs::PosePtr &msg)
{
  desiredPosition[0] = msg->position.x;
  desiredPosition[1] = msg->position.y;
  desiredPosition[2] = msg->position.z;
}

void callback(const geometry_msgs::TransformStampedPtr &msg)
{
  error[0] = msg->transform.translation.x;
  error[1] = msg->transform.translation.y;
  error[2] = -msg->transform.translation.z;
}

void stateCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  pose = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "correction");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Subscriber subscriber = nh_private.subscribe("/visual_servoing/error_transform", 1, callback);
  ros::Subscriber subscriber_2 = nh_private.subscribe("/visual_servoing/next_pose", 1, stateCallback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  ros::Rate rate(50.0);

  pose.header.frame_id = "world";

  while (ros::ok())
  {
    pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
