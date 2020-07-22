#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <cmath>

double randMToN(double m, double n)
{
  return m + (rand() / (RAND_MAX / (n - m)));
}

Eigen::Vector3d planeFromPoints(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double& dist_)
{
  Eigen::Vector3d p1p2 = p2 - p1;
  Eigen::Vector3d p1p3 = p3 - p1;

  Eigen::Vector3d normal = p1p2.cross(p1p3);
  normal = normal.normalized();
  dist_ = normal.dot(p1);
  return normal;
}

double distP(Eigen::Vector3d normal, Eigen::Vector3d point)
{
  double distance = normal.dot(point);
  return distance;
}

bool isInFrustum(Eigen::MatrixXd near_plane, Eigen::MatrixXd far_plane_, Eigen::Vector3d test_point,
                 Eigen::Vector3d trans)
{
  Eigen::Vector3d normal;
  Eigen::Vector3d p1, p2, p3;
  double dd, dist;

  p1[0] = far_plane_(0, 0);
  p1[1] = far_plane_(1, 0);
  p1[2] = far_plane_(2, 0);

  p2[0] = far_plane_(0, 1);
  p2[1] = far_plane_(1, 1);
  p2[2] = far_plane_(2, 1);

  p3[0] = far_plane_(0, 2);
  p3[1] = far_plane_(1, 2);
  p3[2] = far_plane_(2, 2);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);
  dist = distP(normal, test_point) - dd;
  // std::cout << "far " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }
  // 2.

  p1[0] = near_plane(0, 0);
  p1[1] = near_plane(1, 0);
  p1[2] = near_plane(2, 0);

  p3[0] = near_plane(0, 1);
  p3[1] = near_plane(1, 1);
  p3[2] = near_plane(2, 1);

  p2[0] = near_plane(0, 2);
  p2[1] = near_plane(1, 2);
  p2[2] = near_plane(2, 2);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);

  dist = distP(normal, test_point) - dd;
  // std::cout << "close " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }
  // left
  p1[0] = near_plane(0, 3);
  p1[1] = near_plane(1, 3);
  p1[2] = near_plane(2, 3);

  p3[0] = near_plane(0, 2);
  p3[1] = near_plane(1, 2);
  p3[2] = near_plane(2, 2);

  p2[0] = far_plane_(0, 2);
  p2[1] = far_plane_(1, 2);
  p2[2] = far_plane_(2, 2);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);

  dist = distP(normal, test_point) - dd;
  // std::cout << "left " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }
  // right
  p1[0] = near_plane(0, 0);
  p1[1] = near_plane(1, 0);
  p1[2] = near_plane(2, 0);

  p2[0] = near_plane(0, 1);
  p2[1] = near_plane(1, 1);
  p2[2] = near_plane(2, 1);

  p3[0] = far_plane_(0, 0);
  p3[1] = far_plane_(1, 0);
  p3[2] = far_plane_(2, 0);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);

  dist = distP(normal, test_point) - dd;
  // std::cout << "right " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }
  // top
  p1[0] = near_plane(0, 0);
  p1[1] = near_plane(1, 0);
  p1[2] = near_plane(2, 0);

  p3[0] = near_plane(0, 3);
  p3[1] = near_plane(1, 3);
  p3[2] = near_plane(2, 3);

  p2[0] = far_plane_(0, 0);
  p2[1] = far_plane_(1, 0);
  p2[2] = far_plane_(2, 0);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);

  dist = distP(normal, test_point) - dd;
  // std::cout << "top " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }
  // bottom
  p1[0] = near_plane(0, 2);
  p1[1] = near_plane(1, 2);
  p1[2] = near_plane(2, 2);

  p3[0] = near_plane(0, 1);
  p3[1] = near_plane(1, 1);
  p3[2] = near_plane(2, 1);

  p2[0] = far_plane_(0, 1);
  p2[1] = far_plane_(1, 1);
  p2[2] = far_plane_(2, 1);

  p1 = p1 + trans;
  p2 = p2 + trans;
  p3 = p3 + trans;
  normal = planeFromPoints(p1, p2, p3, dd);

  dist = distP(normal, test_point) - dd;
  // std::cout << "bottom " << dist << std::endl;
  if (dist < 0.0)
  {
    return false;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle node_handle;
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  // create 6 bounding planes
  double min_distance = 0.5;
  double max_distance = 1.5;

  double pitch = 0.1;
  double tan_half_horizontal_fov = std::tan(M_PI / 2 / 2.0);
  double tan_half_vertical_fov = std::tan(M_PI / 3.0 / 2.0);

  Eigen::MatrixXd far_plane_ = Eigen::MatrixXd::Zero(3, 4);
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

  Eigen::Matrix3d roty;
  roty = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

  Eigen::Matrix3d rotz;
  // rotz = Eigen::AngleAxisd(M_PI / 3 - M_PI, Eigen::Vector3d::UnitZ());

  ros::Rate r(30);
  Eigen::Vector3d ff(2, 3, 2), fu(1, 2, 1);
  // bool a = fu.array() < ff.array();

  std::cout << "bigger smaller " << (fu.array() < ff.array()).all() << std::endl;
  double angle_step = 0.1;
  double radius_step = 0.1;
  double min_radius = 0.2;
  double max_radius = 2.0;
  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  Eigen::Vector3d tp(0.0, 0.0, 2.0);
  int count = 0;
  for (double r = max_radius; r >= min_radius; r -= radius_step)
  {
    for (double theta = M_PI / 4.0; theta <= M_PI / 2; theta += angle_step)
    {
      for (double phi = 0; phi < M_PI * 2.0; phi += angle_step)
      {
        Eigen::Vector3d test;
        double ddx = r * sin(theta) * cos(phi);
        double ddy = r * sin(theta) * sin(phi);
        double ddz = r * cos(theta);

        // is in frustum
        // test[0] = origin[0] + ddx;
        // test[1] = origin[1] + ddy;
        // test[2] = origin[2] + ddz;
        test[0] = ddx;
        test[1] = ddy;
        test[2] = ddz;
        double yaw = atan2(ddy, ddx);

        rotz = Eigen::AngleAxisd(yaw - M_PI, Eigen::Vector3d::UnitZ());
        Eigen::MatrixXd np = rotz * roty * near_plane;
        Eigen::MatrixXd fp = rotz * roty * far_plane_;

        if (isInFrustum(np, fp, tp, test))
        {
          count++;
          // std::cout << "ddx ddy ddz " << test << std::endl;
          // std::cout << "r " << r << std::endl;
          break;
        }
        if (count == 200)
        {
          // std::cout << "ddx ddy ddz " << test << std::endl;
          // std::cout << "r " << r << std::endl;
        }
      }
    }
  }

  // double dx = 0.433, dy = 0.25, dz = 0.866;
  double dx = 1.41421, dy = 0.0, dz = 1.41421;

  rotz = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ());
  // rotz = Eigen::AngleAxisd(M_PI / 4 - M_PI, Eigen::Vector3d::UnitZ());
  roty = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY());
  // std::cout << "counter " << count << std::endl;
  // near_plane = roty * rotz * near_plane;
  // far_plane_ = roty * rotz * far_plane_;
  near_plane = rotz * roty * near_plane;

  // std::cout << "before " << std::endl;
  // std::cout << far_plane_ << std::endl;
  far_plane_ = rotz * roty * far_plane_;
  // std::cout << "after " << std::endl;
  // std::cout << far_plane_ << std::endl;
  if (isInFrustum(near_plane, far_plane_, tp, Eigen::Vector3d(dx, dy, dz)))
  {
    // std::cout << "pols" << std::endl;
  }

  double gain = 0.0;
  // double min_distance = params_[1];
  double half_horizontal_fov = M_PI / 2 / 2;
  double half_vertical_fov = M_PI / 3 / 2;

  double resolution = 0.05 * 2;
  double step_size = atan(resolution / 1);
  Eigen::Vector3d origin2(dx, dy, dz);
  visualization_msgs::Marker point;
  // std::cout << "hv vv " << tan(-half_horizontal_fov) * min_distance << " " << tan(half_horizontal_fov) * min_distance
  //          << std::endl;
  for (double hv = -half_horizontal_fov; hv <= half_horizontal_fov; hv += step_size)
  {
    for (double vv = -half_vertical_fov; vv <= half_vertical_fov; vv += step_size)
    {
      Eigen::Vector3d start_point(min_distance, tan(hv) * min_distance, tan(vv) * min_distance);
      Eigen::Vector3d end_point(max_distance, tan(hv) * max_distance, tan(vv) * max_distance);

      start_point = rotz * roty * start_point + origin2;
      end_point = rotz * roty * end_point + origin2;
      // std::cout << start_point << std::endl;
      geometry_msgs::Point p;
      p.x = start_point[0];
      p.y = start_point[1];
      p.z = start_point[2];
      point.points.push_back(p);
      p.x = end_point[0];
      p.y = end_point[1];
      p.z = end_point[2];
      point.points.push_back(p);
    }
  }

  while (ros::ok())
  {
    // std::cout << " what " << std::endl;

    point.header.frame_id = "world";
    point.header.stamp = ros::Time::now();
    point.ns = "point";
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.id = 1;
    point.type = visualization_msgs::Marker::POINTS;

    visualization_msgs::Marker line_strip1;
    line_strip1.header.frame_id = "world";
    line_strip1.header.stamp = ros::Time::now();
    line_strip1.ns = "line1";
    line_strip1.action = visualization_msgs::Marker::ADD;
    line_strip1.pose.orientation.w = 1.0;
    line_strip1.id = 1;
    line_strip1.type = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker line_strip2;
    line_strip2.header.frame_id = "world";
    line_strip2.header.stamp = ros::Time::now();
    line_strip2.ns = "line2";
    line_strip2.action = visualization_msgs::Marker::ADD;
    line_strip2.pose.orientation.w = 1.0;
    line_strip2.id = 2;
    line_strip2.type = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker line_strip3;
    line_strip3.header.frame_id = "world";
    line_strip3.header.stamp = ros::Time::now();
    line_strip3.ns = "line3";
    line_strip3.action = visualization_msgs::Marker::ADD;
    line_strip3.pose.orientation.w = 1.0;
    line_strip3.id = 3;
    line_strip3.type = visualization_msgs::Marker::LINE_STRIP;

    visualization_msgs::Marker line_strip4;
    line_strip4.header.frame_id = "world";
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

    geometry_msgs::Point p;
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

    p.x = far_plane_(0, 0) + dx;
    p.y = far_plane_(1, 0) + dy;
    p.z = far_plane_(2, 0) + dz;
    line_strip1.points.push_back(p);

    p.x = far_plane_(0, 1) + dx;
    p.y = far_plane_(1, 1) + dy;
    p.z = far_plane_(2, 1) + dz;
    line_strip2.points.push_back(p);

    p.x = far_plane_(0, 2) + dx;
    p.y = far_plane_(1, 2) + dy;
    p.z = far_plane_(2, 2) + dz;
    line_strip3.points.push_back(p);

    p.x = far_plane_(0, 3) + dx;
    p.y = far_plane_(1, 3) + dy;
    p.z = far_plane_(2, 3) + dz;
    line_strip4.points.push_back(p);

    marker_pub.publish(line_strip1);
    marker_pub.publish(line_strip2);
    marker_pub.publish(line_strip3);
    marker_pub.publish(line_strip4);
    p.x = tp.x();
    p.y = tp.y();
    p.z = tp.z();
    point.points.push_back(p);
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    point.points.push_back(p);
    point.scale.x = 0.2;
    point.scale.y = 0.2;
    point.color.g = 1.0f;
    point.color.a = 1.0;
    marker_pub.publish(point);

    r.sleep();
    // ros::spinOnce();
    // ros::Duration(1).sleep();
  }
  return 0;
}