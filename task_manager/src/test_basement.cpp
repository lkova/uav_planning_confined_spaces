#include <adaptive_planning/planner_srv.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose_luka", 10);
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(50.0);

  // wait for FCU connection
  // while(ros::ok() && !current_state.connected){
  //     ros::spinOnce();
  //     rate.sleep();
  // }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "abs_correction";

  while (ros::ok())
  {
    adaptive_planning::planner_srv plannerSrv;
    plannerSrv.request.goal_set = false;

    if (ros::service::call("planner", plannerSrv))
    {
      if (plannerSrv.response.success)
      {
        std::cout << "in " << std::endl;
        for (int i = 0; i < plannerSrv.response.path.size(); i++)
        {
          pose.pose.position = plannerSrv.response.path[i].position;
          pose.pose.orientation = plannerSrv.response.path[i].orientation;
          std::cout << "pose " << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z
                    << std::endl;
          std::cout << "pose " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
                    << pose.pose.orientation.z << " " << pose.pose.orientation.z << std::endl;
          // local_pos_pub.publish(pose);
          pub.publish(pose);
        }
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}