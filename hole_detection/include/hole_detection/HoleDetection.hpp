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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_set>

// OpenCV
#include <math.h>

#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hole_detection/Hole.h"
#include "opencv2/core.hpp"

namespace hole_detection
{
  class HoleDetection
  {
  public:
    HoleDetection();

    virtual ~HoleDetection();

    void setCloud(const pcl::PCLPointCloud2 &cloud_msg);

    pcl::PointXYZ getHoleCoordinates(bool &found);
    std::vector<std::vector<Eigen::Vector3d>> getHoleTargetFrame();
    std::vector<hole_detection::Hole> getHolesMessage();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getClouds();
    bool isPointUpdated();
    void loadAllTemplates();

    bool pub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c;

    bool debug_;
    bool is_front_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    bool ismynteye_;
    float bias_;
    std::string manhole_path_;
    std::string big_hole_path_;

  private:
    struct Template
    {
      Eigen::VectorXd tempx;
      Eigen::VectorXd tempy;
      double angle;
      double scale_x;
    };

    struct Plane
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
      Eigen::Affine3d R;
      double z_plane;
      Eigen::Vector3d normal;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloudData(const pcl::PCLPointCloud2 &cloud_msg);

    void planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double &z_plane, Eigen::Affine3d &rotation);

    void multiPlaneSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Plane> &planes);

    void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void createImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat &image, double z_plane);

    void publishHole(Eigen::Vector3d &point, double angle, int type, Eigen::Vector3d &plane_normal);
    cv::Point2d projectCloudTo2d(const pcl::PointXYZ xyz, double min_x, double min_y, double resolution, double z_plane);

    void findInMap(pcl::PointXYZ &p1, pcl::PointXYZ &p2, cv::Point p_max, cv::Point p_min);
    bool findDifference(std::vector<cv::Point> curve);
    void findCurveInMap(std::vector<cv::Point> &curve);
    void deleteFunction(std::vector<cv::Point> curve);
    bool getAngle(std::vector<cv::Point> pts);
    void multiTest();
    void findHole(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double z_plane, Eigen::Affine3d rotation, bool &found,
                  Eigen::Vector3d &plane_normal);

    std::vector<std::vector<cv::Point2f>> project2dToCloud(std::vector<cv::Point> contour);
    void visualizeMatch(const std::vector<cv::Point> &contour, const Template &hole,
                        std::vector<std::vector<cv::Point>> &match, double z_plane);

    double matchShapes(std::vector<cv::Point> contour, Template &hole, double z_plane);

    bool checkFreeSpace(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ hole);
    void biggestDist(std::vector<cv::Point> contour);

    void loadTemplate();
    void loadTemplateBigHole();
    // visualization function
    void holeToTargetFrame(std::vector<cv::Point> contour, double z_plane, Eigen::Affine3d R);

    pcl::PointXYZ hole_local_coordinates_;

    bool is_point_updated;
    int imgw_;
    int imgh_;
    double yyy_;
    double xxx_;
    int scale_it_;
    int kernel_size_;

    std::mutex g_i_mutex;
    std::vector<cv::Point> template_contour_;
    std::map<std::pair<int, int>, pcl::PointXYZ> map_;
    uint kernel_resolution_;
    bool defined_;

    Template hole_circle_;
    Template hole_big_;
    std::vector<std::vector<Eigen::Vector3d>> hole_results_;

    std::vector<hole_detection::Hole> hole_msgs_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  };
} // namespace hole_detection