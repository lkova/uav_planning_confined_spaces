#include <chrono>
#include <thread>

#include <adaptive_planning/planner_srv.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <cmath>

Eigen::Vector3d down_hole;
Eigen::Vector3d front_hole;
Eigen::Vector3d drone_position;
ros::Publisher trajectory_pub;
int down_found = 0;
int front_found = 0;

void callbackCloudDown(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  for (pcl::PointXYZ point : msg->points)
  {
    double diff = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));

    if (diff < 0.75)
    {
      // replan
      // ROS_INFO("TOO CLOSE DOWN");
    }
  }
}

void callbackCloudFront(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  for (pcl::PointXYZ point : msg->points)
  {
    double diff = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));

    if (diff < 0.75)
    {
      // replan
      // ROS_INFO("TOO CLOSE FRONT");
    }
  }
}

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
      down_hole[0] = p2.point.x;
      down_hole[1] = p2.point.y;
      down_hole[2] = p2.point.z;
      ROS_INFO("doslo down %lf %lf %lf", down_hole.x(), down_hole.y(), down_hole.z());
      down_found = 1;
      front_found = 0;
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
      front_hole[0] = p2.point.x;
      front_hole[1] = p2.point.y;
      front_hole[2] = p2.point.z;
      if (drone_position.z() < 7)
        front_found = 1;
      ROS_INFO("doslo FRONT %lf %lf %lf", front_hole.x(), front_hole.y(), front_hole.z());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
}

void poseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  drone_position[0] = msg->position.x;
  drone_position[1] = msg->position.y;
  drone_position[2] = msg->position.z;
}

void moveFromHole(geometry_msgs::Pose p, double dx, double dy, double dz, bool is_down)
{
  int rep = 0;
  while (rep < 5)
  {
    rep++;
    ROS_INFO("wait for detection");
    ros::spinOnce();
    ros::Duration(0.2).sleep();
  }

  double discrete_size = dx + dy + dz;
  double step = 0.2;
  double sign = 1;

  if (discrete_size < 0)
  {
    sign = -1;
  }

  discrete_size = abs(discrete_size);

  if (is_down == true)
  {
    p.position.x = down_hole.x();
    p.position.y = down_hole.y();
    p.position.z = down_hole.z();
    dz = 1 * sign;
  }
  else
  {
    p.position.x = front_hole.x();
    p.position.y = front_hole.y();
    p.position.z = front_hole.z();
    dx = 1 * sign;
  }

  for (double i = 0.0; i < discrete_size; i += step)
  {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    Eigen::Vector3d desired_position(p.position.x + dx * i, p.position.y + dy * i, p.position.z + dz * i);
    double desired_yaw = 0.0;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("hello %lf", i);
  }
}

bool planPath(geometry_msgs::Pose p)
{
  adaptive_planning::planner_srv plannerSrv;
  plannerSrv.request.goal = p;
  plannerSrv.request.goal_set = true;

  if (ros::service::call("planner", plannerSrv))
  {
    ROS_INFO("planner srv size %d", plannerSrv.response.path.size());

    if (plannerSrv.response.success)
    {
      if (plannerSrv.response.path.size() > 0)
      {
        double last_yaw = 0.0;
        for (int i = 0; i < plannerSrv.response.path.size(); i++)
        {
          trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
          trajectory_msg.header.stamp = ros::Time::now();

          Eigen::Vector3d desired_position(plannerSrv.response.path[i].position.x,
                                           plannerSrv.response.path[i].position.y,
                                           plannerSrv.response.path[i].position.z);
          tf::Pose pose;
          tf::poseMsgToTF(plannerSrv.response.path[i], pose);
          // double yaw = tf::getYaw(pose.getRotation());
          double ddx = plannerSrv.response.path[i].position.x - drone_position.x();
          double ddy = plannerSrv.response.path[i].position.y - drone_position.y();
          // double yaw = atan2(ddy, ddx);
          double yaw = 0.0;
          if (i % 2 == 0)
          {
            yaw = 3.14;
          }
          // if (i == plannerSrv.response.path.size() - 1 || i % 3 != 0)
          //  yaw = 0.0;
          last_yaw = yaw;

          ROS_INFO("yaw you OK %lf", last_yaw);
          mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, yaw, &trajectory_msg);

          trajectory_pub.publish(trajectory_msg);
          ros::spinOnce();
          ros::Duration(0.1).sleep();
        }
      }
      return true;
    }
  }
  return false;
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

  ros::Subscriber sub_cloud_down = nh.subscribe("/firefly/vi_sensor/camera_depth/depth/points", 1, callbackCloudDown);

  ros::Subscriber sub_cloud_front =
      nh.subscribe("/firefly/vi_sensor2/camera_depth/depth/points", 1, callbackCloudFront);

  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("firefly/command/trajectory", 10);
  ROS_INFO("Started hovering example.");

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("/firefly/vi_sensor/ground_truth/pose", 1, poseCallback);

  trajectory_msgs::MultiDOFJointTrajectory samples_array;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
  int n_seq = 0;
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  std::vector<Eigen::Vector3d> waypoints;

  /*waypoints.push_back(Eigen::Vector3d(6.5, 0, 11));
  waypoints.push_back(Eigen::Vector3d(5, 0, 6.5));
  waypoints.push_back(Eigen::Vector3d(5.2, 0.5, 6.2));
  waypoints.push_back(Eigen::Vector3d(5.5, 1, 6.2));
  waypoints.push_back(Eigen::Vector3d(5.0, -0.5, 6.0));
  waypoints.push_back(Eigen::Vector3d(10, 0, 5.5));

  // back
  waypoints.push_back(Eigen::Vector3d(5.0, -0.5, 6.0));
  waypoints.push_back(Eigen::Vector3d(5.5, 1, 6.2));
  waypoints.push_back(Eigen::Vector3d(5.2, 0.5, 6.2));
  waypoints.push_back(Eigen::Vector3d(6.5, 0, 11));*/

  /*waypoints.push_back(Eigen::Vector3d(5.0, 0, 9.0));
  waypoints.push_back(Eigen::Vector3d(6.3, 0, 6.05));
  waypoints.push_back(Eigen::Vector3d(6.0, -0.5, 5));
  waypoints.push_back(Eigen::Vector3d(7, -0.6, 5));
  waypoints.push_back(Eigen::Vector3d(6.3, 0, 6.05));
  waypoints.push_back(Eigen::Vector3d(8, 0, 5));*/

  waypoints.push_back(Eigen::Vector3d(5.0, 0, 9.0));
  waypoints.push_back(Eigen::Vector3d(5.5, -1, 9));
  waypoints.push_back(Eigen::Vector3d(6.0, -2, 10));
  waypoints.push_back(Eigen::Vector3d(5, 1, 9.5));
  waypoints.push_back(Eigen::Vector3d(6, 3, 10));
  // waypoints.push_back(Eigen::Vector3d(8, 0, 5));

  double hdz = 7;
  double hfx = 8.2;

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    for (Eigen::Vector3d point : waypoints)
    {
      ROS_INFO("drone position %lf %lf", drone_position.z(), drone_position.x());

      int sign_down = (drone_position.z() - hdz) > 0 ? 1 : -1;
      int sign_front = (drone_position.x() - hfx) > 0 ? 1 : -1;
      int hwz;
      if (point.z() - hdz > 0)
        hwz = 1;
      else
        hwz = -1;

      int hwx;
      if (point.x() - hfx > 0)
        hwx = 1;
      else
        hwx = -1;

      geometry_msgs::Pose p;
      double dx = 0, dy = 0, dz = 0;
      bool down = false;
      if (hwz != sign_down || hwx != sign_front && 0)
      {
        if (hwz != sign_down)
        {
          if (down_found == 1)
          {
            ROS_INFO("down found");
            p.position.x = down_hole.x();
            p.position.y = down_hole.y();
            p.position.z = down_hole.z() + 1 * sign_down;
            dx = 0;
            dz = -2.0 * sign_down;
            down = true;
          }
          else
          {
            while (1)
            {
              ROS_INFO("down not found");
            }
          }
        }
        else if (hwx != sign_front)
        {
          if (front_found == 1)
          {
            p.position.x = front_hole.x() + 0.8 * sign_front;
            p.position.y = front_hole.y();
            p.position.z = front_hole.z();
            dz = 0;
            dx = -2.0 * sign_front;
            down = false;
          }
          else
          {
            while (1)
            {
              ROS_INFO("front not found");
            }
          }
        }

        int repeat = 0;
        bool great_success = planPath(p);

        /*while (!planPath(p))
        {
          ROS_INFO("planning %d", repeat);
          repeat++;
          if (repeat == 3)
            break;
        }
        if (repeat == 3)
        {
          ROS_INFO("plan failed");
        }
        else
        {
          great_success = true;
        }*/

        if (great_success)
        {
          moveFromHole(p, dx, dy, dz, down);
          if (down)
          {
            dx = -0.3;
            trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
            trajectory_msg.header.stamp = ros::Time::now();
            Eigen::Vector3d desired_position(p.position.x + dx, p.position.y + dy, p.position.z + dz);
            double desired_yaw = 0.0;
            mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
            trajectory_pub.publish(trajectory_msg);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
          }
        }
      }
      // else
      //{
      p.position.x = point.x();
      p.position.y = point.y();
      p.position.z = point.z();
      int repeat = 0;
      planPath(p);
      /*while (!planPath(p))
      {
        ROS_INFO("planning %d", repeat);
        repeat++;
        if (repeat == 3)
          break;
      }
      if (repeat == 3)
      {
        ROS_INFO("plan failed");
      }*/
      //}

      ros::spinOnce();
      ros::Duration(1).sleep();
    }

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  return 0;
}
