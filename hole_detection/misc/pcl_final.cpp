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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
geometry_msgs::PointStamped final_points;

double depth;
int max_n;
ros::Publisher pub;

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> EdgeFinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double depth,
                                                             int max_n)

{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(10);
  n.compute(*normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
  oed.setInputNormals(normals);
  oed.setInputCloud(cloud);
  oed.setDepthDisconThreshold(depth);
  oed.setMaxSearchNeighbors(max_n);
  oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED |
                  oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
  pcl::PointCloud<pcl::Label> labels;
  std::vector<pcl::PointIndices> label_indices;
  oed.compute(labels, label_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges(new pcl::PointCloud<pcl::PointXYZ>),
      occluded_edges(new pcl::PointCloud<pcl::PointXYZ>), nan_boundary_edges(new pcl::PointCloud<pcl::PointXYZ>),
      high_curvature_edges(new pcl::PointCloud<pcl::PointXYZ>), rgb_edges(new pcl::PointCloud<pcl::PointXYZ>);

  copyPointCloud(*cloud, label_indices[0].indices, *nan_boundary_edges);
  copyPointCloud(*cloud, label_indices[1].indices, *occluding_edges);
  copyPointCloud(*cloud, label_indices[2].indices, *occluded_edges);
  copyPointCloud(*cloud, label_indices[3].indices, *high_curvature_edges);
  copyPointCloud(*cloud, label_indices[4].indices, *rgb_edges);
  const int point_size = 2;
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  std::cout << nan_boundary_edges->size() << std::endl;
  std::cout << occluding_edges->size() << std::endl;
  std::cout << occluded_edges->size() << std::endl;
  std::cout << high_curvature_edges->size() << std::endl;
  std::cout << rgb_edges->size() << std::endl;

  // viewer->addPointCloud(cloud, "original point cloud");
  /*viewer->addPointCloud<pcl::PointXYZ>(nan_boundary_edges, "nan_boundary_edges");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
  "nan_boundary_edges");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f,
  "nan_boundary_edges");*/
  /*viewer->addPointCloud<pcl::PointXYZ>(high_curvature_edges, "high_curvature_edges");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size,
  "high_curvature_edges"); viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
  0.0f, 1.0f, 0.0f, "high_curvature_edges");
  //viewer->addPointCloudNormals(cloud_with_normals, 10, 0.05, "normals");
  //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  while (!viewer->wasStopped())
  {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }*/
  return high_curvature_edges;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_filtered);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  // Convert to the templated PointCloud

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points."
            << std::endl;

  // Write the downsampled version to disk
  // pcl::PCDWriter writer;
  // writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int)cloud_filtered->points.size();
  // While 30% of the original cloud is still there

  while (cloud_filtered->points.size() > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points."
              << std::endl;

    /*std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
    */
    // Create the filtering object
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered.swap(cloud_f);
    i++;
  }
  /*extract.filter(*cloud_p);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_filtered);
  chull.setAlpha(10.0);
  chull.reconstruct(*cloud_hull);*/

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.3);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered);
  return cloud_filtered;
}

void SurfaceSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal> mls_points)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  // pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

  mls.setComputeNormals(true);

  // Set parameters
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);

  // Reconstruct
  mls.process(mls_points);
}

void BoundingBoxFinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloud, pcaCentroid);
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // Final transform
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);  // Quaternions are a way to do rotations
                                                             // https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  // This viewer has 4 windows, but is only showing images in one of them as written here.
  /*boost::shared_ptr<pcl::visualization::PCLVisualizer> visu(new pcl::visualization::PCLVisualizer("3D Viewer"));

  int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
  visu->createViewPort(0.0, 0.5, 0.5, 1.0, mesh_vp_1);
  visu->createViewPort(0.5, 0.5, 1.0, 1.0, mesh_vp_2);
  visu->createViewPort(0.0, 0, 0.5, 0.5, mesh_vp_3);
  visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
  visu->addPointCloud(cloud, "bboxedCloud", mesh_vp_3);
  visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z -
  minPoint.z, "bbox", mesh_vp_3); while (!visu->wasStopped())
  {
      visu->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }*/

  // std::cout << "prvi " << maxPoint.x - minPoint.x << "drugi " << maxPoint.y - minPoint.y << "treci " << maxPoint.z -
  // minPoint.z << std::endl;
  double length = maxPoint.x - minPoint.x;
  double height = maxPoint.y - minPoint.y;
  double depth = maxPoint.z - minPoint.z;
  double x = pcaCentroid(0, 0);
  double y = pcaCentroid(1, 0);
  double z = pcaCentroid(2, 0);

  std::cout << "kurac " << x << " " << y << " " << z << std::endl;
  final_points.header.frame_id = "pelican/vi_sensor/camera_depth_optical_center_link";
  final_points.header.stamp = ros::Time::now();
  final_points.point.x = x;
  final_points.point.y = y;
  final_points.point.z = z;
  pub.publish(final_points);
}

void cloud_db(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  ROS_INFO("got_it");
  pcl::PCLPointCloud2 *cloud_pcl = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*cloud_msg, *cloud_pcl);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloud_pcl, *cloud);

  // FilterPointCloud(cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = EdgeFinder(cloud, depth, max_n);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.3);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered);

  // NEW
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud_filtered);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);
  mls.process(*mls_points);
  BoundingBoxFinder(mls_points);
}
int main(int argc, char *argv[])
{
  depth = atof(argv[1]);
  max_n = int(atof(argv[2]));
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCLPointCloud2 cloud_blob;
  // pcl::io::loadPCDFile("/home/luka/bun000_Structured_A.pcd", cloud_blob);

  // double K = atof(argv[1]);

  // pcl::io::loadPCDFile("/home/luka/drone_ws/src/real.pcd", cloud_blob);
  // pcl::fromPCLPointCloud2(cloud_blob, *cloud);
  // pcl::io::loadPCDFile("/home/luka/drone_ws/src/real.pcd", *cloud);
  // pcl::PointCloud<pcl::PointNormal> mls_points;
  // SurfaceSegmentation(cloud, mls_points);
  // std::cout << mu << " " << radius << std::endl;

  /*pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  gp3.setMu(mu);
  gp3.setSearchRadius(radius);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();*/
  // pcl::io::saveVTKFile("mesh.vtk", triangles);
  // viewer->addPolygonMesh(triangles, "meshes", 0);

  // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
  /*viewer->addPointCloud<pcl::PointCloud<pcl::PointXYZ>>(mls_points, "pick");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, mls_points, 10, 0.05, "Normals");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  while (!viewer->wasStopped())
  {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
  */
  ros::init(argc, argv, "cool_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pelican/vi_sensor_down/camera_depth/depth/points", 1, cloud_db);
  pub = nh.advertise<geometry_msgs::PointStamped>("pointy", 1000);

  // pcl::io::savePCDFile("/home/luka/picka.pcd", mls_points);
  /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  //viewer->addPointCloud(cloud_filtered, "cloud");
  viewer->addPointCloud(mls_points.makeShared(), "wtf");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped())
  {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }*/

  ros::spin();
  return 0;
}