#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class cloudHandler
{
public:
  cloudHandler()
  {
    pcl_sub = nh.subscribe("/pelican/vi_sensor/camera_depth/depth/points", 10, &cloudHandler::cloudCB, this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_segmented", 1);
    ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);

    coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
  }

  void cloudCB(const sensor_msgs::PointCloud2 &input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
    pcl::fromROSMsg(input, cloud);
    /*pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;

    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setInputCloud(cloud.makeShared());
    segmentation.segment(*inliers, coefficients);
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    ros_coefficients.header.stamp = input.header.stamp;
    coef_pub.publish(ros_coefficients);
    pcl_msgs::PointIndices ros_inliers;
    pcl_conversions::fromPCL(*inliers, ros_inliers);
    ros_inliers.header.stamp = input.header.stamp;
    ind_pub.publish(ros_inliers);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(cloud_segmented);
    sensor_msgs::PointCloud2 output;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ana(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud_ana = cloud_segmented;
    mls.setInputCloud(cloud_ana);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    // Reconstruct
    mls.process(mls_points);

    pcl::toROSMsg(mls_points, output);
    pcl_pub.publish(output);
    bool printIt = cloud.isOrganized();
    auto he = cloud.size();*/
    pcl::io::savePCDFile("real.pcd", cloud);
    //std::cout << "organized " << printIt << "size" << he << std::endl;
    //std::vector<pcl::PointXYZ> data = cloud.points;
  }

protected:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Publisher pcl_pub, ind_pub, coef_pub;
};
main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_planar_segmentation");
  cloudHandler handler;
  ros::spin();
  return 0;
}