#include "adaptive_planning/Planner.hpp"

namespace planner
{
Planner::Planner(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
{
  // all the initializations and parameters
  // target_frame_ = "abs_correction";
  target_frame_ = "world";
  octomap_sub_ = nodeHandle.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &Planner::octomapCallback, this);
  pose_sub_ = nodeHandle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/firefly/odometry_sensor1/pose_with_covariance", 1, &Planner::poseCovarianceCallback, this);
  // pose_sub_ = nodeHandle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/rovio/pose_with_covariance_stamped",
  // 1,
  //                                                                             &Planner::poseCovarianceCallback,
  //                                                                             this);
  plannerService_ = nodeHandle_.advertiseService("planner", &Planner::plannerServiceCallback, this);
  marker_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  if (!readParameters())
  {
    ROS_ERROR("Could not start the planner. Parameters missing!");
  }
  setCameraParameters();

  Eigen::Vector4d params(2, 1, params_.cam_horizontal_, params_.cam_vertical_);

  path_finder_ = new PathFinder(far_plane_, roty_, params);
}

Planner::~Planner()
{
}

bool Planner::readParameters()
{
  // if (!nodeHandle_.getParam("camera_pitch", params_.camPitch_))
  //   return false;
  nodeHandle_.getParam("front", camera_type_front);
  nodeHandle_.getParam("camera/horizontal", params_.cam_horizontal_);
  nodeHandle_.getParam("camera/vertical", params_.cam_vertical_);
  nodeHandle_.getParam("camera/min_distance", params_.min_distance_);
  nodeHandle_.getParam("camera/max_distance", params_.max_distance_);

  camera_type_front = true;
  if (camera_type_front == true)
  {
    params_.cam_pitch_ = 0.1;
  }
  else
  {
    params_.cam_pitch_ = M_PI / 2;
  }
  params_.cam_horizontal_ = M_PI / 2;
  params_.cam_vertical_ = M_PI / 3;
  // params_.cam_horizontal_ = params_.cam_horizontal_ / 180.0 * M_PI;
  // params_.cam_vertical_ = params_.cam_vertical_ / 180.0 * M_PI;
  return true;
}

void Planner::setCameraParameters()
{
  // create 6 bounding planes
  double min_distance = params_.min_distance_;
  double max_distance = params_.max_distance_;
  min_distance = 1;
  max_distance = 2;
  double pitch = params_.cam_pitch_;
  double tan_half_horizontal_fov = std::tan(params_.cam_horizontal_ / 2.0);
  double tan_half_vertical_fov = std::tan(params_.cam_vertical_ / 2.0);

  far_plane_ = Eigen::MatrixXd::Zero(3, 4);
  Eigen::MatrixXd near_plane = Eigen::MatrixXd::Zero(3, 4);

  near_plane(0, 0) = min_distance;
  near_plane(1, 0) = min_distance * tan_half_horizontal_fov;
  near_plane(2, 0) = min_distance * tan_half_vertical_fov;

  near_plane(0, 1) = min_distance;
  near_plane(1, 1) = min_distance * tan_half_horizontal_fov;
  near_plane(2, 1) = -min_distance * tan_half_vertical_fov;

  near_plane(0, 2) = min_distance;
  near_plane(1, 2) = -min_distance * tan_half_horizontal_fov;
  near_plane(2, 2) = -min_distance * tan_half_vertical_fov;

  near_plane(0, 3) = min_distance;
  near_plane(1, 3) = -min_distance * tan_half_horizontal_fov;
  near_plane(2, 3) = min_distance * tan_half_vertical_fov;

  far_plane_(0, 0) = max_distance;
  far_plane_(1, 0) = max_distance * tan_half_horizontal_fov;
  far_plane_(2, 0) = max_distance * tan_half_vertical_fov;

  far_plane_(0, 1) = max_distance;
  far_plane_(1, 1) = max_distance * tan_half_horizontal_fov;
  far_plane_(2, 1) = -max_distance * tan_half_vertical_fov;

  far_plane_(0, 2) = max_distance;
  far_plane_(1, 2) = -max_distance * tan_half_horizontal_fov;
  far_plane_(2, 2) = -max_distance * tan_half_vertical_fov;

  far_plane_(0, 3) = max_distance;
  far_plane_(1, 3) = -max_distance * tan_half_horizontal_fov;
  far_plane_(2, 3) = max_distance * tan_half_vertical_fov;

  roty_ = Eigen::AngleAxisd(params_.cam_pitch_, Eigen::Vector3d::UnitY());
}

bool Planner::plannerServiceCallback(adaptive_planning::planner_srv::Request &req,
                                     adaptive_planning::planner_srv::Response &res)
{
  ros::Time time = ros::Time::now();
  res.path.clear();

  std::vector<Eigen::Vector3d> path;

  Eigen::Vector3d goal_pose;
  bool goal_set = req.goal_set;
  // goal_set = false;

  if (goal_set)
  {
    goal_pose[0] = req.goal.position.x;
    goal_pose[1] = req.goal.position.y;
    goal_pose[2] = req.goal.position.z;
    std::cout << "goal is set " << goal_pose << std::endl;
    std::vector<Eigen::Vector3d> path;
    path_finder_->goalSelector(goal_pose, goal_set);
    path_finder_->plan(path);

    int u = 0;
    for (Eigen::Vector3d e : path)
    {
      geometry_msgs::Pose pose;
      pose.position.x = e.x();
      pose.position.y = e.y();
      pose.position.z = e.z();
      res.path.push_back(pose);
      res.success = true;
      ROS_INFO("path %lf %lf %lf", res.path[u].position.x, res.path[u].position.y, res.path[u].position.z);
      u++;
    }
  }
  if (!goal_set)
  {
    std::vector<Eigen::Vector4d> path;
    res.success = path_finder_->poseToExplore(path);
    for (Eigen::Vector4d e : path)
    {
      visualizeFrustum(e);
      // visualizeUnknownPoints();
      geometry_msgs::Pose pose;
      pose.position.x = e.x();
      pose.position.y = e.y();
      pose.position.z = e.z();
      tf::Quaternion q;
      q.setRPY(0, 0, e.w());
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();

      res.path.push_back(pose);
      // ROS_INFO("path %lf %lf %lf", res.path[u].position.x,
      // res.path[u].position.y, res.path[u].position.z); u++;
    }
  }
}

// void Planner::poseCallback(const geometry_msgs::PoseConstPtr &msg)
// {
//   path_finder_->setPose(msg);
// }

void Planner::poseCovarianceCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  path_finder_->setPose(msg);
}

void Planner::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
  path_finder_->setOctomap(msg);
}

void Planner::visualizeUnknownPoints()
{
  std::vector<Eigen::Vector3d> points;
  points = path_finder_->getVisPoints();
  visualization_msgs::Marker point;
  point.header.frame_id = target_frame_;
  point.header.stamp = ros::Time::now();
  point.ns = "point2";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1.0;
  point.id = 1;
  point.type = visualization_msgs::Marker::POINTS;

  // std::cout << "points size " << points.size() << std::endl;
  for (auto po : points)
  {
    geometry_msgs::Point p;
    p.x = po.x();
    p.y = po.y();
    p.z = po.z();
    point.points.push_back(p);
  }
  marker_pub_.publish(point);
  ros::spinOnce();
  ros::Duration(0.5).sleep();
}

void Planner::visualizeFrustum(Eigen::Vector4d pose)
{
  double dx = pose.x(), dy = pose.y(), dz = pose.z();

  Eigen::Matrix3d rotz;
  rotz = Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ());
  geometry_msgs::Point p;
  Eigen::MatrixXd near_plane = Eigen::MatrixXd::Zero(3, 4);
  Eigen::MatrixXd far_plane;
  // std::cout << "cp " << params_.camPitch_ << std::endl;
  // std::cout << "before " << std::endl;
  // std::cout << far_plane_ << std::endl;
  far_plane = rotz * roty_ * far_plane_;
  // std::cout << "after " << std::endl;
  // std::cout << far_plane << std::endl;

  std::vector<Eigen::Vector3d> points;
  points = path_finder_->getVisPoints();
  visualization_msgs::Marker point3;
  point3.header.frame_id = target_frame_;
  point3.header.stamp = ros::Time::now();
  point3.ns = "point3";
  point3.action = visualization_msgs::Marker::ADD;
  point3.pose.orientation.w = 1.0;
  point3.id = 5;
  point3.type = visualization_msgs::Marker::POINTS;
  point3.scale.x = 0.2;
  point3.scale.y = 0.2;
  point3.color.g = 1.0f;
  point3.color.a = 1.0;

  // std::cout << "points size " << points.size() << std::endl;
  for (auto po : points)
  {
    p.x = po.x();
    p.y = po.y();
    p.z = po.z();
    point3.points.push_back(p);
  }
  marker_pub_.publish(point3);

  visualization_msgs::Marker point;

  point.header.frame_id = target_frame_;
  point.header.stamp = ros::Time::now();
  point.ns = "point";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1.0;
  point.id = 1;
  point.type = visualization_msgs::Marker::POINTS;

  visualization_msgs::Marker line_strip1;
  line_strip1.header.frame_id = target_frame_;
  line_strip1.header.stamp = ros::Time::now();
  line_strip1.ns = "line1";
  line_strip1.action = visualization_msgs::Marker::ADD;
  line_strip1.pose.orientation.w = 1.0;
  line_strip1.id = 1;
  line_strip1.type = visualization_msgs::Marker::LINE_STRIP;

  visualization_msgs::Marker line_strip2;
  line_strip2.header.frame_id = target_frame_;
  line_strip2.header.stamp = ros::Time::now();
  line_strip2.ns = "line2";
  line_strip2.action = visualization_msgs::Marker::ADD;
  line_strip2.pose.orientation.w = 1.0;
  line_strip2.id = 2;
  line_strip2.type = visualization_msgs::Marker::LINE_STRIP;

  visualization_msgs::Marker line_strip3;
  line_strip3.header.frame_id = target_frame_;
  line_strip3.header.stamp = ros::Time::now();
  line_strip3.ns = "line3";
  line_strip3.action = visualization_msgs::Marker::ADD;
  line_strip3.pose.orientation.w = 1.0;
  line_strip3.id = 3;
  line_strip3.type = visualization_msgs::Marker::LINE_STRIP;

  visualization_msgs::Marker line_strip4;
  line_strip4.header.frame_id = target_frame_;
  line_strip4.header.stamp = ros::Time::now();
  line_strip4.ns = "line4";
  line_strip4.action = visualization_msgs::Marker::ADD;
  line_strip4.pose.orientation.w = 1.0;
  line_strip4.id = 4;
  line_strip4.type = visualization_msgs::Marker::LINE_STRIP;

  line_strip1.scale.x = 0.1;
  line_strip1.color.b = 1.0;
  line_strip1.color.a = 1.0;

  line_strip2.scale.x = 0.1;
  line_strip2.color.b = 1.0;
  line_strip2.color.a = 1.0;

  line_strip3.scale.x = 0.1;
  line_strip3.color.b = 1.0;
  line_strip3.color.a = 1.0;

  line_strip4.scale.x = 0.1;
  line_strip4.color.b = 1.0;
  line_strip4.color.a = 1.0;

  /*p.x = dx;
  p.y = dy;
  p.z = dz;
  line_strip1.points.push_back(p);
  line_strip2.points.push_back(p);
  line_strip3.points.push_back(p);
  line_strip4.points.push_back(p);*/

  p.x = near_plane(0, 0) + dx;
  p.y = near_plane(1, 0) + dy;
  p.z = near_plane(2, 0) + dz;
  line_strip1.points.push_back(p);

  p.x = near_plane(0, 1) + dx;
  p.y = near_plane(1, 1) + dy;
  p.z = near_plane(2, 1) + dz;
  line_strip2.points.push_back(p);

  p.x = near_plane(0, 2) + dx;
  p.y = near_plane(1, 2) + dy;
  p.z = near_plane(2, 2) + dz;
  line_strip3.points.push_back(p);

  p.x = near_plane(0, 3) + dx;
  p.y = near_plane(1, 3) + dy;
  p.z = near_plane(2, 3) + dz;
  line_strip4.points.push_back(p);

  p.x = far_plane(0, 0) + dx;
  p.y = far_plane(1, 0) + dy;
  p.z = far_plane(2, 0) + dz;
  line_strip1.points.push_back(p);

  p.x = far_plane(0, 1) + dx;
  p.y = far_plane(1, 1) + dy;
  p.z = far_plane(2, 1) + dz;
  line_strip2.points.push_back(p);

  p.x = far_plane(0, 2) + dx;
  p.y = far_plane(1, 2) + dy;
  p.z = far_plane(2, 2) + dz;
  line_strip3.points.push_back(p);

  p.x = far_plane(0, 3) + dx;
  p.y = far_plane(1, 3) + dy;
  p.z = far_plane(2, 3) + dz;
  line_strip4.points.push_back(p);

  marker_pub_.publish(line_strip1);
  marker_pub_.publish(line_strip2);
  marker_pub_.publish(line_strip3);
  marker_pub_.publish(line_strip4);
  p.x = dx;
  p.y = dy;
  p.z = dz;
  point.points.push_back(p);
  point.scale.x = 0.2;
  point.scale.y = 0.2;
  point.color.g = 1.0f;
  point.color.a = 1.0;
  marker_pub_.publish(point);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  //}
}
}  // namespace planner