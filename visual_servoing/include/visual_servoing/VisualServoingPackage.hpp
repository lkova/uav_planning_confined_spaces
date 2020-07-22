#pragma once

#include <hole_detection/Hole.h>

#include "visual_servoing/VisualServoing.hpp"

// PCL
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <chrono>
#include <ctime>
#include <functional>

namespace visual_servoing
{
class VisualServoingPackage
{
public:
  VisualServoingPackage(ros::NodeHandle &nodeHandle);

  virtual ~VisualServoingPackage();

private:
  bool readParameters();
  // void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void imageCallback(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::ImageConstPtr &depth_msg);
  void holeCallback(const hole_detection::Hole &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  void publishTransform();

  ros::Subscriber test;
  message_filters::Subscriber<sensor_msgs::Image> subscribe_raw_image_;
  message_filters::Subscriber<sensor_msgs::Image> subscribe_depth_image_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  ros::NodeHandle &nodeHandle_;
  // ros::Subscriber subscriber_image_;
  ros::Publisher publisher_;
  ros::Publisher publisher_transform_;
  ros::Subscriber subscriber_hole_;
  ros::Subscriber drone_pose_subscriber_;
  ros::Subscriber camera_info_sub_;
  bool iscamsub_;
  ros::Publisher corrected_pose_;
  nav_msgs::Odometry reference_odom_;
  nav_msgs::Odometry odom_;
  std::string image_topic_;
  std::string depth_image_topic_;
  std::string target_frame_;
  std::string pose_topic_;
  std::string camera_info_topic_;
  std::string base_link_frame_;
  std::string camera_frame_;
  bool is_simulation_;
  bool is_ref_image_;
  cv::Mat ref_image_;
  visual_servoing::VisualServoing visual_servoing;
};
}  // namespace visual_servoing