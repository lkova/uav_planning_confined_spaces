/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
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

#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("firefly/command/trajectory", 5);
  ROS_INFO("Started exploration");

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

  double dt = 1.0;
  std::string ns = ros::this_node::getName();

  static int n_seq = 0;

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(2.0).sleep();

  // This is the initialization motion, necessary that the known free space allows the planning
  // of initial paths.

  // Start planning: The planner is called and the computed path sent to the controller.
  int iteration = 0;
  double current = 0.0;

  while (ros::ok())
  {
    double desired_yaw = -3.14 + (rand() / (RAND_MAX / (3.14 + 3.14)));
    std::cout << "des " << desired_yaw << std::endl;
    double sgn = 1.0;
    if (current > desired_yaw)
    {
      sgn = -1.0;
    }
    while (1)
    {
      Eigen::Vector3d desired_position(4, 0.0, 10.0);

      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();

      double diff = fabs(current - desired_yaw);
      std::cout << "diff " << diff << std::endl;

      if (diff < 0.7)
      {
        current = desired_yaw;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, current, &trajectory_msg);
        trajectory_pub.publish(trajectory_msg);
        ros::Duration(1.0).sleep();
        break;
      }
      current += 0.7 * sgn;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, current, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);

      // ROS_INFO("yes");
      std::cout << "cur " << current << std::endl;

      ros::Duration(1).sleep();
    }
    ROS_INFO("new round");
    ros::Duration(1.0).sleep();
  }
}
