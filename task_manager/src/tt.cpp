#include <chrono>
#include <thread>

#include <adaptive_planning/planner_srv.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <cmath>

double x;
double y;
double z;
int down_found = 0;
int front_found = 0;
double px;
double q;
double r;

void callbackDown(const geometry_msgs::PointStampedConstPtr msg)
{
  if (down_found == 0)
  {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
      geometry_msgs::PointStamped p_world;
      geometry_msgs::PointStamped p2;
      listener.lookupTransform("world", "firefly/vi_sensor/camera_depth_optical_center_link", ros::Time(0), transform);
      listener.transformPoint("world", *msg, p2);
      x = p2.point.x;
      y = p2.point.y;
      z = p2.point.z;
      ROS_INFO("doslo down %lf %lf %lf", x, y, z);
      down_found = 1;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void callbackFront(const geometry_msgs::PointStampedConstPtr msg)
{
  if (front_found == 0)
  {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
      geometry_msgs::PointStamped p_world;
      geometry_msgs::PointStamped p2;
      listener.lookupTransform("world", "firefly/vi_sensor2/camera_depth_optical_center_link", ros::Time(0), transform);
      listener.transformPoint("world", *msg, p2);
      x = p2.point.x;
      y = p2.point.y;
      z = p2.point.z;
      front_found = 1;
      ROS_INFO("doslo FRONT %lf %lf %lf", x, y, z);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void poseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  px = msg->position.x;
  q = msg->position.y;
  r = msg->position.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");

  ros::Subscriber sub_down =
      nh.subscribe<geometry_msgs::PointStamped>("/firefly/detectionDown/pointy", 1, callbackDown);
  ros::Subscriber sub_front =
      nh.subscribe<geometry_msgs::PointStamped>("/firefly/detectionFront/pointy", 1, callbackFront);

  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("firefly/command/trajectory", 10);
  ROS_INFO("Started hovering example.");

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/firefly/vi_sensor/ground_truth/pose", 1, poseCallback);

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  int n_seq = 0;
  ros::Duration(3.0).sleep();

  while (ros::ok())
  {
    adaptive_planning::planner_srv plannerSrv;
    if (down_found == 1 || front_found == 1)
    {
      geometry_msgs::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = z;
      plannerSrv.request.goal = p;
      plannerSrv.request.goal_set = true;
      ROS_INFO("WTF %lf %lf %lf", p.position.x, p.position.y, p.position.z);
      if (ros::service::call("planner", plannerSrv))
      {
        ROS_INFO("planner srv size %d", plannerSrv.response.path.size());

        if (plannerSrv.response.success)
        {
          if (plannerSrv.response.path.size() > 0)
          {
            for (int i = 0; i < plannerSrv.response.path.size(); i++)
            {
              /*ROS_INFO("pls %lf %lf %lf", plannerSrv.response.path[i].position.x,
                       plannerSrv.response.path[i].position.y, plannerSrv.response.path[i].position.z);
              samples_array.header.seq = n_seq;
              samples_array.header.stamp = ros::Time::now();
              samples_array.header.frame_id = "world";
              samples_array.points.clear();
              tf::Pose pose;
              tf::poseMsgToTF(plannerSrv.response.path[i], pose);
              double yaw = tf::getYaw(pose.getRotation());
              trajectory_point.position_W.x() = plannerSrv.response.path[i].position.x;
              trajectory_point.position_W.y() = plannerSrv.response.path[i].position.y;
              // Add offset to account for constant tracking error of controller
              trajectory_point.position_W.z() = plannerSrv.response.path[i].position.z;
              tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
              trajectory_point.setFromYaw(tf::getYaw(quat));
              mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
              samples_array.points.push_back(trajectory_point_msg);
              trajectory_pub.publish(samples_array);*/

              trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
              trajectory_msg.header.stamp = ros::Time::now();

              Eigen::Vector3d desired_position(plannerSrv.response.path[i].position.x,
                                               plannerSrv.response.path[i].position.y,
                                               plannerSrv.response.path[i].position.z);
              tf::Pose pose;
              tf::poseMsgToTF(plannerSrv.response.path[i], pose);
              // double yaw = tf::getYaw(pose.getRotation());
              double ddx = plannerSrv.response.path[i].position.x - px;
              double ddy = plannerSrv.response.path[i].position.y - q;
              // double yaw = atan2(ddy, ddx);
              double yaw = 0.0;
              ROS_INFO("yaw you OK %lf", yaw * 180.0 / 3.14);
              mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, yaw, &trajectory_msg);

              trajectory_pub.publish(trajectory_msg);
              ros::spinOnce();
              ros::Duration(0.2).sleep();
            }
            ROS_INFO("step is done");
            ros::spinOnce();
            ros::Duration(2.0).sleep();

            double dx = 0.0;
            double dz = 0.0;
            double dy = 0.0;
            if (front_found == 1 && down_found != 1)
            {
              front_found = 2;
              dx = 2.0;
            }
            if (down_found == 1)
            {
              down_found = 2;
              dz = -2.0;
              dx = -0.4;
              dy = -0.03;
              int a = 0;
              while (a < 3)
              {
                ROS_INFO("WTF %lf %lf %lf", px, q, r);
                a++;
                ros::spinOnce();
                ros::Duration(1.0).sleep();
              }
              trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
              trajectory_msg.header.stamp = ros::Time::now();
              ROS_INFO("WTF %lf %lf %lf", px, q + dy, r + dz);
              Eigen::Vector3d desired_position(px, q + dy, r + dz);
              mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, 0, &trajectory_msg);

              trajectory_pub.publish(trajectory_msg);
            }

            int a = 0;
            while (a < 3)
            {
              ROS_INFO("WTF %lf %lf %lf", px, q, r);
              a++;
              ros::spinOnce();
              ros::Duration(1.0).sleep();
            }
            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();
            ROS_INFO("WTF %lf %lf %lf", px, q, r + dz);
            Eigen::Vector3d desired_position(px + dx, q, r);
            double desired_yaw = 0.0;
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

            trajectory_pub.publish(trajectory_msg);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
          }
        }
      }
    }
    ros::spinOnce();
    ros::Duration(3.0).sleep();
  }
  // ros::Duration(1.0).sleep();
  return 0;
}
