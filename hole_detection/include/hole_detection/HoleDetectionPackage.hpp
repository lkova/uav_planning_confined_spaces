#pragma once

#include "hole_detection/HoleDetection.hpp"

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
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "hole_detection/Hole.h"

namespace hole_detection
{
class HoleDetectionPackage
{
public:
  HoleDetectionPackage(ros::NodeHandle &nodeHandle);

  virtual ~HoleDetectionPackage();

private:
  bool readParameters();

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void poseCallback(const geometry_msgs::PointStampedPtr &msg);
  void getTransform(tf::StampedTransform &transform, bool &found);
  void visualizeHoles(std::vector<std::vector<Eigen::Vector3d>> holes, std::vector<hole_detection::Hole> msgs);
  void publishHolesTF(std::vector<hole_detection::Hole> holes);

  ros::NodeHandle &nodeHandle_;

  ros::Subscriber subscriber_;
  ros::Subscriber pose_subscriber_;

  ros::Publisher publisher_;
  ros::Publisher publisher_holes;

  ros::Publisher publish_segmented_plane_;

  hole_detection::HoleDetection holeDetection_;
  geometry_msgs::PointStampedPtr drone_pose_;

  ros::Publisher marker_pub;

  std::string subscriberTopic_;
  std::string cameraFrameTopic_;
  std::string target_frame_id_;
  bool is_mynteye_;
};

}  // namespace hole_detection
