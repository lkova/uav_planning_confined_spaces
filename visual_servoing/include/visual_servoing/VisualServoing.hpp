#pragma once

#include <geometry_msgs/PointStamped.h>
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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_set>

// OpenCV
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Sparse>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
namespace visual_servoing
{
class VisualServoing
{
public:
  struct Params
  {
    double fx;
    double fy;
    double cx;
    double cy;
    bool iscutoff;
    bool debug;
    int match_type;
    int patch_size;
  } params;

  VisualServoing();

  virtual ~VisualServoing();

  void setImage(cv::Mat &image, cv::Mat &depth_image, bool isref = false);

  void execute();

  void visualize();

  Eigen::Matrix4f getTransformation();

private:
  cv::Ptr<cv::Feature2D> detector_;

  struct ImageFeatures
  {
    cv::Mat image = cv::Mat();
    std::vector<cv::KeyPoint> keypoints = std::vector<cv::KeyPoint>();
    cv::Mat descriptors = cv::Mat();
    cv::Mat depth_image = cv::Mat();
  };

  ImageFeatures reference_image_;
  ImageFeatures image_;
  void projectTo3d(const cv::Point2f &point2d, cv::Point3f &point3d);
  void findFeatures(ImageFeatures &image);
  void matchImages();
  void matchImages2();
  void ransacTest(const std::vector<cv::DMatch> matches, const std::vector<cv::KeyPoint> &matched1,
                  const std::vector<cv::KeyPoint> &matched2, std::vector<cv::DMatch> &goodMatches,
                  std::vector<cv::KeyPoint> &inliers1, std::vector<cv::KeyPoint> &inliers2, double distance,
                  double confidence, double minInlierRatio);

  Eigen::Matrix4f estimateRigidTransform(Eigen::Matrix3Xf &points_reference, Eigen::Matrix3Xf &points_target);

  void customDrawMatches(const cv::Mat &im1, const std::vector<cv::Point2f> &pts_1, const cv::Mat &im2,
                         const std::vector<cv::Point2f> &pts_2, cv::Mat &outImage,
                         const std::string msg = std::string(""), std::vector<uchar> status = std::vector<uchar>(),
                         bool enable_lines = true, bool enable_points = true, bool enable_test = true);

  void ransac(Eigen::Matrix3Xf &points_reference, Eigen::Matrix3Xf &points_target);
  void ransacPatch(Eigen::Matrix3Xf &points_reference, Eigen::Matrix3Xf &points_target,
                   std::vector<Eigen::Matrix3Xf> &reference_vector, std::vector<Eigen::Matrix3Xf> &target_vector);
  std::vector<cv::DMatch> good_matches_;
  Eigen::Matrix4f transformation_;
};
}  // namespace visual_servoing