#pragma once

#include "adaptive_planning/PathFinder.hpp"
#include "adaptive_planning/planner_srv.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <ompl/config.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

namespace planner
{
struct Params
{
  double cam_pitch_;
  double cam_horizontal_;
  double cam_vertical_;
  std::vector<std::vector<Eigen::Vector3d> > camBoundNormals_;
  double min_distance_;
  double max_distance_;
};

class Planner
{
public:
  Planner(ros::NodeHandle &nodeHandle);
  virtual ~Planner();

private:
  bool readParameters();
  void setCameraParameters();
  void calculateCameraFrustum(Eigen::Vector3d &min_plane, Eigen::Vector3d &max_plane);

  void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
  // void poseCallback(const geometry_msgs::PoseConstPtr &msg);
  void poseCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool plannerServiceCallback(adaptive_planning::planner_srv::Request &req,
                              adaptive_planning::planner_srv::Response &res);
  void visualizeFrustum(Eigen::Vector4d pose);
  void visualizeUnknownPoints();

  ros::NodeHandle nodeHandle_;
  ros::Subscriber octomap_sub_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer plannerService_;
  ros::Publisher marker_pub_;

  PathFinder *path_finder_;
  std::string target_frame_;
  Params params_;
  Eigen::MatrixXd far_plane_;
  Eigen::Matrix3d roty_;
  bool camera_type_front;
};

}  // namespace planner