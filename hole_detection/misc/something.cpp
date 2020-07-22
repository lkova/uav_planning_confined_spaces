#include <math.h>
#include <pcl/2d/morphology.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "pcl/features/fpfh.h"

int main(int argc, char* argv[])
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::io::loadPCDFile("/home/luka/drone_ws/src/my_pcl_tutorial/src/point.pcd", *cloud);

  pcl::Morphology<pcl::PointXYZRGB> morphology;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("/home/luka/drone_ws/src/my_pcl_tutorial/src/point.pcd", *input_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr structuring_element_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  morphology.structuringElementCircular(*structuring_element_cloud, 3);
  // morphology.setStructuringElement(structuring_element_cloud);
  // morphology.structuring_element_ = *structuring_element_cloud;
  morphology.operator_type = pcl::Morphology<pcl::PointXYZRGB>::DILATION_GRAY;
  // morphology.applyMorphologicalOperation(*output_cloud, *input_cloud);
  morphology.applyMorphologicalOperation(*input_cloud);

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(input_cloud);
  while (!viewer.wasStopped())
  {
  }
  return 0;
}