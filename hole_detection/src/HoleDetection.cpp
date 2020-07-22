#include "hole_detection/HoleDetection.hpp"

namespace hole_detection
{
HoleDetection::HoleDetection()
{
  kernel_resolution_ = 7;
  hole_results_ = std::vector<std::vector<Eigen::Vector3d>>();

  if (ismynteye_)
  {
    bias_ = 0.8;
  }
  else
  {
    bias_ = 1.0;
  }
}

HoleDetection::~HoleDetection()
{
}

void HoleDetection::loadAllTemplates()
{
  loadTemplate();
  loadTemplateBigHole();
}
void HoleDetection::setCloud(const pcl::PCLPointCloud2 &cloud_msg)
{
  hole_results_.clear();
  hole_msgs_.clear();
  clouds_.clear();
  cloud_c = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_ = loadCloudData(cloud_msg);
  cloud_filtered_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_p_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_c = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pub = false;
  if (!ismynteye_)
    filterCloud(cloud_);
  if (cloud_->size() > 0)
  {
    // pcl::io::loadPCDFile<pcl::PointXYZ>("/home/luka/picka_full.pcd", *cloud_);
    pcl::copyPointCloud(*cloud_, *cloud_c);
    // pcl::io::savePCDFileASCII("/home/luka/picka_full.pcd", *cloud_);
    multiTest();
  }

  // ROS_INFO("before");
  // double z_plane = 0;
  // Eigen::Affine3d rotation;
  // planeSegmentation(cloud_, z_plane, rotation);
  // ROS_INFO("after");
  // findHole(cloud_p_, z_plane, rotation, found);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HoleDetection::loadCloudData(const pcl::PCLPointCloud2 &cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_msg, *cloud);

  // if (cloud->size() > 0)
  // pcl::io::savePCDFileASCII("/home/luka/justcloud.pcd", *cloud);

  return cloud;
}

// CHANGE DATA LOADING TO YAML OR BINARY FILE
void HoleDetection::loadTemplate()
{
  // Eigen::MatrixXd m(279,2);
  // Eigen::VectorXd tempx(279);
  // Eigen::VectorXd tempy(279);
  hole_circle_.tempx = Eigen::VectorXd::Zero(279);
  hole_circle_.tempy = Eigen::VectorXd::Zero(279);

  // std::ifstream myfile("/home/luka/outputFile.txt");
  std::cout << manhole_path_ << std::endl;
  std::ifstream myfile(manhole_path_);

  int i = 0;
  if (myfile.is_open())
  {
    int a, b;
    while (myfile >> a >> b)
    {
      template_contour_.push_back(cv::Point(a, b));
      // m.resize(i + 1, 2);
      // m(i, 0) = a;
      // m(i, 0) = b;
      hole_circle_.tempx[i] = a;
      hole_circle_.tempy[i] = b;
      i++;
    }
    // std::cout << "FOUND " << template_contour_.size() << std::endl;
    hole_circle_.tempx = hole_circle_.tempx.array() - hole_circle_.tempx.mean();
    hole_circle_.tempy = hole_circle_.tempy.array() - hole_circle_.tempy.mean();
    hole_circle_.scale_x = 0.6;
    hole_circle_.angle = 90;
    myfile.close();
  }

  else
    std::cout << "Unable to open file" << std::endl;
}

void HoleDetection::loadTemplateBigHole()
{
  // Eigen::MatrixXd m(279,2);
  // Eigen::VectorXd tempx(279);
  // Eigen::VectorXd tempy(279);

  int n;
  if (ismynteye_)
  {
    n = 136;
  }
  else
  {
    n = 343;
  }
  hole_big_.tempx = Eigen::VectorXd::Zero(n);
  hole_big_.tempy = Eigen::VectorXd::Zero(n);

  std::ifstream myfile(big_hole_path_);

  int i = 0;
  if (myfile.is_open())
  {
    int a, b;
    while (myfile >> a >> b)
    {
      template_contour_.push_back(cv::Point(a, b));
      // m.resize(i + 1, 2);
      // m(i, 0) = a;
      // m(i, 0) = b;
      hole_big_.tempx[i] = a;
      hole_big_.tempy[i] = b;
      i++;
      // std::cout << "wtfff " << a << " " << b << std::endl;
    }
    myfile.close();
    hole_big_.scale_x = 1.3;
    // hole_big_.scale_x = 1.19;
    if (ismynteye_)
      hole_big_.angle = 5;
    else
      hole_big_.angle = 22;
  }
  else
    std::cout << "Unable to open file big hole" << std::endl;
}

void HoleDetection::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 4.5);
  pass.filter(*cloud);

  // std::cout << "before size " << cloud->points.size() << std::endl;
  double resolution_before = cloud->points.size();
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.03f, 0.03f, 0.03f);
  sor.filter(*cloud);

  // std::cout << "after size " << cloud->points.size() << std::endl;
  double resolution_after = cloud->points.size();
  kernel_resolution_ = 0;

  if (!isnan(resolution_before) || resolution_before > 0)
  {
    kernel_resolution_ = sqrt(resolution_before / resolution_after) * 2;
  }

  if (cloud->size() > 0)
  {
    pcl::copyPointCloud(*cloud, *cloud_c);
    // pcl::io::savePCDFileASCII("/home/luka/anali.pcd", *cloud);
  }
}

void HoleDetection::planeSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double &z_plane,
                                      Eigen::Affine3d &rotation)
{
  // separate this function later on

  // ROS_INFO("segmentation");
  pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndicesPtr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new
  // pcl::PointCloud<pcl::PointXYZ>); pcl::PointCloud<pcl::PointXYZ>::Ptr
  // cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud, *cloud_filtered_);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(500);

  seg.setInputCloud(cloud_filtered_);
  seg.segment(*inliers, *coefficients);
  Eigen::Vector3d norm;
  if (coefficients->values.size() > 0)
  {
    std::cout << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
              << coefficients->values[2] << " " << coefficients->values[3] << std::endl;
    norm[0] = coefficients->values[0];
    norm[1] = coefficients->values[1];
    norm[2] = coefficients->values[2];
    z_plane = -coefficients->values[3];
  }

  if (inliers->indices.size() != 0)
  {
    extract.setInputCloud(cloud_filtered_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p_);
    /*std::cerr << "PointCloud representing the planar component: " <<
     cloud_p_->width * cloud_p_->height
      << " data points." << std::endl;*/
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered_.swap(cloud_f);
    // std::cout << "cloud filtered info " << cloud_filtered_->width << " " <<
    // cloud_filtered_->height << std::endl; std::cout << "cloud filtered info "
    // << cloud_->width << " " << cloud_->height << std::endl;
  }
  // kurac_ = *cloud_f;
  // cloud_p_ = cloud_p;
  // cloud_filtered_ = cloud_filtered;

  /// DELETE
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_p_);

  // Create an empty kdtree representation, and pass it to the normal estimation
  // object. Its content will be filled inside the object, based on the given
  // input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*cloud_normals);

  if (cloud_p_->points.size() > 0)
  {
    // ROTATION
    double n = cloud_p_->points.size();
    Eigen::Vector3d normal1 = Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d axis = normal1.cross(norm);
    double angle = -acos(normal1.dot(norm));
    axis = axis.normalized();
    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
    transform_2.rotate(Eigen::AngleAxisd(angle, axis));

    // Print the transformation
    // printf("\nMethod #2: using an Affine3d\n");
    // std::cout << transform_2.matrix() << std::endl;
    rotation = transform_2;
    Eigen::Vector3d normal2(norm);
    Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(normal1, normal2);
    Eigen::Matrix3d R = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d R2 = R.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::transformPointCloud(*cloud_p_, *transformed_cloud, transform_2);
    pcl::copyPointCloud(*transformed_cloud, *cloud_c);
    // pcl::io::savePCDFileASCII("/home/luka/normal.pcd", *cloud_p_);
    cloud_p_ = transformed_cloud;
    // pcl::io::savePCDFileASCII("/home/luka/trans.pcd", *transformed_cloud);
  }
}

void HoleDetection::multiPlaneSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Plane> &planes)
{
  pcl::ModelCoefficientsPtr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndicesPtr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud(*cloud, *cloud_filtered_);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);

  seg.setDistanceThreshold(0.05);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(500);

  int i = 0, nr_points = (int)cloud_filtered_->points.size();

  while (cloud_filtered_->points.size() > 0.3 * nr_points && planes.size() < 14)
  {
    seg.setInputCloud(cloud_filtered_);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_filtered_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_p_);

    // calculate rotation

    Eigen::Vector3d norm;

    Eigen::Vector3d normal1 = Eigen::Vector3d(0, 0, 1);
    Plane plane;
    if (coefficients->values.size() > 0)
    {
      // std::cout << "Model coefficients: " << coefficients->values[0] << " "
      // << coefficients->values[1] << " "
      //          << coefficients->values[2] << " " << coefficients->values[3]
      //          << std::endl;
      norm[0] = coefficients->values[0];
      norm[1] = coefficients->values[1];
      norm[2] = coefficients->values[2];
      plane.normal = norm;
    }

    Eigen::Vector3d normal2(norm);
    Eigen::Vector3d axis = normal1.cross(normal2);
    double angle = -acos(normal1.dot(norm));
    axis = axis.normalized();
    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
    transform_2.rotate(Eigen::AngleAxisd(angle, axis));

    // Print the transformation
    // printf("\nMethod #2: using an Affine3d\n");
    // std::cout << transform_2.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_p_, *transformed_cloud, transform_2);
    cloud_p_ = transformed_cloud;

    // pcl::io::savePCDFileASCII("/home/luka/tu.pcd", *transformed_cloud);
    // pcl::io::savePCDFileASCII("/home/luka/nu.pcd", *cloud_p_);

    // save plane and its rotation

    plane.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr somecl(new
    // pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_p_, *plane.cloud);
    plane.R = transform_2;
    plane.z_plane = -coefficients->values[3];

    planes.push_back(plane);

    // continue filtering the cloud
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_filtered_.swap(cloud_f);
    i++;
  }
}

void HoleDetection::multiTest()
{
  std::vector<Plane> planes;
  multiPlaneSegmentation(cloud_, planes);
  std::cout << "planes number " << planes.size() << std::endl;

  for (Plane plane : planes)
  {
    // std::cout << "planes number " << plane.cloud->points.size() << std::endl;
    bool found = false;

    findHole(plane.cloud, plane.z_plane, plane.R, found, plane.normal);

    if (found)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud =
          pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::copyPointCloud(*plane.cloud, *temp_cloud);
      pcl::transformPointCloud(*temp_cloud, *temp_cloud, plane.R.inverse());
      clouds_.push_back(temp_cloud);
    }
  }

  // int i = obrisi_;
  // if (planes.size() > 0)
  // {
  //   bool found = false;
  //   // pcl::io::savePCDFileASCII("/home/luka/picka.pcd", *planes[0].cloud);
  //   findHole(planes[i].cloud, planes[i].z_plane, planes[i].R, found, planes[i].normal);
  // }
}
void HoleDetection::findHole(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double z_plane, Eigen::Affine3d rotation,
                             bool &found, Eigen::Vector3d &plane_normal)
{
  cv::Mat image;
  createImage(cloud, image, z_plane);

  std::vector<cv::Point> ppc;
  if (!defined_)
    return;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  // hole_type = 0 -> circular
  // hole_type = 1 -> big
  int hole_type = -1;
  int default_kernel = 3;

  if (kernel_resolution_ > 11)
  {
    kernel_resolution_ = 12;
  }
  if (ismynteye_)
    kernel_resolution_ = 12;
  int morph_size = kernel_resolution_ + default_kernel;

  cv::Mat kk = cv::Mat::ones(morph_size, morph_size, CV_8U);

  cv::Mat element_default = getStructuringElement(cv::MORPH_RECT, cv::Size(default_kernel, default_kernel));
  cv::dilate(image, image, kk);

  cv::dilate(image, image, element_default, cv::Point(-1, -1), 7);

  cv::erode(image, image, element_default, cv::Point(-1, -1), 7);

  cv::erode(image, image, kk);

  // cv::imwrite("/home/luka/pussAFTER.jpg", image);
  cv::Mat imgCanny;
  cv::Canny(image,     // input image
            imgCanny,  // output image
            30,        // low threshold
            200);      // high threshold

  // cv::imwrite("/home/luka/canny.jpg", imgCanny);

  cv::findContours(imgCanny, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1);

  cv::Scalar color(255, 0, 0);
  cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

  if (debug_)
  {
    cv::imshow("imgCanny", imgCanny);
    cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::waitKey(2);
  }
  double theee_rate = 0;
  if (contours.size() > 0)
  {
    std::cout << "size " << contours.size() << std::endl;

    double min = -1;
    int bci = -1;
    std::vector<std::vector<cv::Point>> cs2;
    std::vector<cv::Point> cs;

    for (int i = 0; i < contours.size(); i++)
    {
      double area = cv::contourArea(contours[i]);
      // std::cout << "area i " << area << " " << i << std::endl;
      cv::Moments m = cv::moments(contours[i]);
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;

      if (!isnan(cx) && !isnan(cy))
      {
        std::vector<cv::Point> curve;
        // cv::approxPolyDP(contours[i], curve, 3, true);
        // use either poly for actual points
        // double correct_rate = 3;
        double correct_rate1 = 0.0, correct_rate2 = 0.0;

        if (area > 200)
        {
          correct_rate1 = matchShapes(contours[i], hole_circle_, z_plane);
          std::cout << "correct rate it is 1 " << correct_rate1 << std::endl;
          if (correct_rate1 <= 0.70)
          {
            correct_rate2 = matchShapes(contours[i], hole_big_, z_plane);
            std::cout << "correct rate it is 2 " << correct_rate2 << std::endl;
            if (correct_rate2 > 0.70)
            {
              hole_type = 1;
              bci = i;
              break;
            }
          }
          else
          {
            bci = i;
            hole_type = 0;

            break;
          }
        }
      }
    }

    if (bci != -1)
    {
      // publish found
      found = true;
      cv::Moments m = cv::moments(contours[bci]);
      double cx = m.m10 / m.m00;
      double cy = m.m01 / m.m00;
      double u20 = m.m20 / m.m00 - cx * cx;
      double u11 = m.m11 / m.m00 - cx * cy;
      double u02 = m.m02 / m.m00 - cy * cy;
      double angle = floor(abs(0.5 * atan2(2 * u11, (u20 - u02)) * 180.0 / M_PI));

      is_point_updated = true;
      pcl::PointXYZ pos;
      pos.x = cx * fabs(z_plane) / fx_ * scale_it_ + xxx_;
      pos.y = cy * fabs(z_plane) / fy_ * scale_it_ + yyy_;
      pos.z = z_plane;

      Eigen::Vector3d tran(pos.x, pos.y, pos.z);
      tran = rotation.inverse() * tran;
      pos.x = tran.x();
      pos.y = tran.y();
      pos.z = tran.z();

      cv::circle(dst, cv::Point(cx, cy), 2, cv::Scalar(0, 0, 255));
      // if (is_point_updated && checkFreeSpace(cloud_, pcl::PointXYZ(pos.x,
      // pos.y, z_plane)))
      static int cc = 0;
      static int bc = 0;

      if (is_point_updated)
      {
        hole_local_coordinates_.x = pos.x;
        hole_local_coordinates_.y = pos.y;
        hole_local_coordinates_.z = pos.z;

        ROS_INFO("valid point %lf %lf %lf", pos.x, pos.y, pos.z);
        publishHole(tran, angle, hole_type, plane_normal);
        cv::Mat dpl = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

        if (debug_)
          cv::drawContours(dpl, contours, bci, color, CV_FILLED);

        // cv::imwrite("/home/luka/c.jpg", dpl);

        if (hole_type == 0)
        {
          ROS_INFO("circle");
          // std::string path = "/home/luka/Desktop/tsave/c/c" +
          // std::to_string(cc) + ".jpg"; cc++; cv::imwrite(path, image);
        }
        else
        {
          ROS_INFO("big");
          // std::string path = "/home/luka/Desktop/tsave/b/b" +
          // std::to_string(bc) + ".jpg"; bc++; cv::imwrite(path, image);
        }

        pub = true;
      }
      else
      {
        is_point_updated = false;
        pub = false;
      }
    }
    else
    {
      is_point_updated = false;
    }

    if (bci != -1)
    {
      std::vector<std::vector<cv::Point>> match;
      if (hole_type == 0)
        visualizeMatch(contours[bci], hole_circle_, match, z_plane);
      if (hole_type == 1)
        visualizeMatch(contours[bci], hole_big_, match, z_plane);

      // holeToTargetFrame(match[0], z_plane, rotation.inverse());
      holeToTargetFrame(contours[bci], z_plane, rotation.inverse());
      if (debug_)
      {
        color[0] = 0;
        color[1] = 0;
        color[2] = 255;
        drawContours(dst, contours, bci, color, 2, 8);
        color[0] = 255;
        color[1] = 255;
        color[1] = 255;
        drawContours(dst, match, -1, color, 2, 8);
        cv::Mat dpl = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
        cv::drawContours(dpl, match, -1, color, CV_FILLED);
        // cv::imwrite("/home/luka/real.jpg", dpl);
        cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
        // Show windows
        cv::imshow("imgOriginal", dst);

        // std::ofstream myfile("/home/luka/bigHoleReal2.txt");

        // int i = 0;
        // cv::Moments m = cv::moments(contours[bci]);
        // int cx = m.m10 / m.m00;
        // int cy = m.m01 / m.m00;
        // if (myfile.is_open())
        // {
        //   for (auto c : contours[bci])
        //   {
        //     int a = c.x - cx;
        //     int b = c.y - cy;
        //     myfile << a << " " << b << std::endl;
        //   }
        //   myfile.close();
        // }
      }
    }
  }
}

void HoleDetection::visualizeMatch(const std::vector<cv::Point> &contour, const Template &hole,
                                   std::vector<std::vector<cv::Point>> &match, double z_plane)
{
  double scalex = hole.scale_x * fx_ / fabs(z_plane) * scale_it_ * bias_;
  double scaley = hole.scale_x * fy_ / fabs(z_plane) * scale_it_;
  double dfx = hole.tempx.maxCoeff() - hole.tempx.minCoeff();
  double scale = scalex / dfx;

  cv::Moments m = cv::moments(contour);
  double cx = m.m10 / m.m00;
  double cy = m.m01 / m.m00;
  double u20 = m.m20 / m.m00 - cx * cx;
  double u11 = m.m11 / m.m00 - cx * cy;
  double u02 = m.m02 / m.m00 - cy * cy;
  double angle = floor(abs(0.5 * atan2(2 * u11, (u20 - u02)) * 180.0 / M_PI));
  double rot_angle = (angle - hole.angle) * M_PI / 180.0;
  rot_angle = -rot_angle;
  std::vector<cv::Point> contour_temp;
  for (int i = 0; i < hole.tempx.size(); i++)
  {
    cv::Point tp;
    double x = hole.tempx[i] * scale;
    double y = hole.tempy[i] * scale;
    tp.x = x * cos(rot_angle) - y * sin(rot_angle) + cx;
    tp.y = x * sin(rot_angle) + y * cos(rot_angle) + cy;
    contour_temp.push_back(tp);
  }
  match.push_back(contour_temp);
}

void HoleDetection::findInMap(pcl::PointXYZ &p1, pcl::PointXYZ &p2, cv::Point p_max, cv::Point p_min)
{
  // Find 2d points in map
  // should be optimized
  // can also be done by approximate calculation back to 3d again

  double diff_bench_max = 100;
  double diff_bench_min = 100;

  for (const auto &pair : map_)
  {
    double diff_max = sqrt(pow(pair.first.first - p_max.x, 2) + pow(pair.first.second - p_max.y, 2));
    double diff_min = sqrt(pow(pair.first.first - p_min.x, 2) + pow(pair.first.second - p_min.y, 2));

    if (diff_bench_max > diff_max)
    {
      diff_bench_max = diff_max;
      p1.x = pair.second.x;
      p1.y = pair.second.y;
      p1.z = pair.second.z;
    }
    if (diff_bench_min > diff_min)
    {
      diff_bench_min = diff_min;
      p2.x = pair.second.x;
      p2.y = pair.second.y;
      p2.z = pair.second.z;
    }
  }
}

void HoleDetection::createImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat &image, double z_plane)
{
  const std::lock_guard<std::mutex> lock(g_i_mutex);
  // change to global variables

  double resolution = 1 / fx_;

  map_ = std::map<std::pair<int, int>, pcl::PointXYZ>();
  float xmin = VTK_FLOAT_MAX;
  float xmax = VTK_FLOAT_MIN;
  float ymin = VTK_FLOAT_MAX;
  float ymax = VTK_FLOAT_MIN;
  float zmin = VTK_FLOAT_MAX;
  float zmax = VTK_FLOAT_MIN;

  defined_ = false;
  // double z_p = 0;
  if (cloud->points.size() > 50)
  {
    // std::cout << "building image " << std::endl;
    for (pcl::PointXYZ point : cloud->points)
    {
      // z_p += point.z;
      xmin = std::min(xmin, (float)point.x);
      xmax = std::max(xmax, (float)point.x);
      ymin = std::min(ymin, (float)point.y);
      ymax = std::max(ymax, (float)point.y);
      zmin = std::min(zmin, (float)point.z);
      zmax = std::max(zmax, (float)point.z);
    }
    // z_p /= cloud->points.size();
    // std::cout << "x " << xmin << " " << xmax << std::endl;
    // std::cout << "y " << ymin << " " << ymax << std::endl;
    // std::cout << "z " << zmin << " " << zmax << std::endl;
    int height = (ymax - ymin) / resolution / fabs(z_plane);
    int width = (xmax - xmin) / resolution / fabs(z_plane);
    xxx_ = xmin;
    yyy_ = ymin;

    scale_it_ = 1;
    if (width > 1280)
    {
      scale_it_ = 1280 / width;
    }
    height = height * scale_it_ + 5;
    width = width * scale_it_ + 5;
    // std::cout << "hei wid " << height << " " << width << std::endl;
    cv::Mat image2(height, width, CV_8U, cv::Scalar(0));
    for (pcl::PointXYZ point : cloud->points)
    {
      cv::Point uv_rect = projectCloudTo2d(point, xmin, ymin, resolution, z_plane);
      image2.at<uchar>(uv_rect.y, uv_rect.x) = 255;
      // map_[std::make_pair(uv_rect.x, uv_rect.y)] = point;
      if (uv_rect.y > height - 1 || uv_rect.y < 0 || uv_rect.x > width - 1 || uv_rect.x < 0)
      {
        std::cout << "h w out of bounds " << height << " " << width << " " << uv_rect << std::endl;
      }
    }
    // std::cout << "z coef normal " << z_plane << " " << z_p << std::endl;
    defined_ = true;
    image = image2;
  }
}

cv::Point2d HoleDetection::projectCloudTo2d(const pcl::PointXYZ xyz, double min_x, double min_y, double resolution,
                                            double z_plane)
{
  // generate uv coordinates
  cv::Point2d uv_rect;
  float az = fabs(z_plane);
  uv_rect.x = (xyz.x - min_x) / resolution / az * scale_it_;
  uv_rect.y = (xyz.y - min_y) / resolution / az * scale_it_;

  return uv_rect;
}

pcl::PointXYZ HoleDetection::getHoleCoordinates(bool &found)
{
  found = pub;
  return hole_local_coordinates_;
}

bool HoleDetection::isPointUpdated()
{
  return is_point_updated;
}

bool HoleDetection::checkFreeSpace(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ hole)
{
  for (pcl::PointXYZ point : cloud->points)
  {
    double diff_x = pow(hole.x - point.x, 2);
    double diff_y = pow(hole.y - point.y, 2);
    double diff_z = pow(hole.z - point.z, 2);
    double r = sqrt(diff_x + diff_y + diff_z);
    if (r < 0.2)
    {
      std::cout << "not in the free space" << std::endl;
      return false;
    }
  }
  return true;
}

double HoleDetection::matchShapes(std::vector<cv::Point> contour, Template &hole, double z_plane)
{
  cv::Moments m = cv::moments(contour);
  double cx = m.m10 / m.m00;
  double cy = m.m01 / m.m00;
  double u20 = m.m20 / m.m00 - cx * cx;
  double u11 = m.m11 / m.m00 - cx * cy;
  double u02 = m.m02 / m.m00 - cy * cy;
  double angle = floor(abs(0.5 * atan2(2 * u11, (u20 - u02)) * 180.0 / M_PI));

  double scalex = hole.scale_x * fx_ / fabs(z_plane) * scale_it_ * bias_;
  double scaley = hole.scale_x * fy_ / fabs(z_plane) * scale_it_;
  double dfx = hole.tempx.maxCoeff() - hole.tempx.minCoeff();
  double scale = scalex / dfx;

  // can be changed
  double error = 0.1 * fx_ / fabs(z_plane) * scale_it_;
  int error_count = 0;
  int j = 0;
  double total_sum = 0;
  // std::cout << "error " << error << std::endl;
  // check difference angle
  double diff_angle = angle - hole.angle;
  int best_good = INT_MIN;
  int bs = 0;
  for (int angle_add = 0; angle_add < 3; angle_add++)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      int good = 0;
      double rot_angle = -(diff_angle + angle_add * sign) * M_PI / 180.0;
      std::unordered_set<int> seen;
      for (cv::Point p : contour)
      {
        double min_error = INT_MAX;
        int min_index = -1;

        for (int i = 0; i < hole.tempx.size(); i++)
        {
          if (seen.find(i) != seen.end())
            continue;

          cv::Point tp;
          double x = hole.tempx[i] * scale;
          double y = hole.tempy[i] * scale;
          tp.x = x * cos(rot_angle) - y * sin(rot_angle) + cx;
          tp.y = x * sin(rot_angle) + y * cos(rot_angle) + cy;
          double diff = sqrt(pow(tp.x - p.x, 2) + pow(tp.y - p.y, 2));
          // std::cout << "diff " << diff << std::endl;
          if (diff < min_error && diff < error)
          {
            min_error = diff;
            min_index = i;
          }
        }
        // std::cout << "why " << min_error << " " << min_index << " " << error
        // << std::endl;
        if (min_index != -1)
        {
          good++;
          seen.insert(min_index);
        }
      }
      if (good > best_good)
      {
        best_good = good;
        bs = seen.size();
      }
    }
  }
  double rot_angle = diff_angle * M_PI / 180.0;
  std::cout << "good " << best_good << " " << contour.size() << " " << bs << std::endl;
  return static_cast<double>(best_good) / static_cast<double>(contour.size());
}

bool HoleDetection::getAngle(std::vector<cv::Point> pts)
{
  int sz = static_cast<int>(pts.size());
  cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
  for (int i = 0; i < pts.size(); i++)
  {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }

  cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                             static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
  // Store the eigenvalues and eigenvectors
  std::vector<cv::Point2d> eigen_vecs(2);
  std::vector<double> eigen_val(2);
  for (int i = 0; i < 2; i++)
  {
    eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
    eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
  }
  double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x);  // orientation in radians
  // std::cout << "angle " << angle << std::endl;
  return false;
}

void HoleDetection::publishHole(Eigen::Vector3d &point, double angle, int type, Eigen::Vector3d &plane_normal)
{
  ros::Time begin = ros::Time::now();
  hole_detection::Hole hole;
  // hole.header.frame_id = "world";
  hole.header.stamp = begin;
  hole.point.x = point.x();
  hole.point.y = point.y();
  hole.point.z = point.z();
  hole.normal.x = plane_normal.x();
  hole.normal.y = plane_normal.y();
  hole.normal.z = plane_normal.z();
  hole.angle = angle;
  if (type == 0)
  {
    hole.type = "manhole";
  }
  else
  {
    hole.type = "passage";
  }

  hole_msgs_.push_back(hole);
  ROS_INFO("hole detected %lf %lf %lf", point.x(), point.y(), point.z());
  ROS_INFO("normal %lf %lf %lf", plane_normal.x(), plane_normal.y(), plane_normal.z());
}

void HoleDetection::holeToTargetFrame(std::vector<cv::Point> contour, double z_plane, Eigen::Affine3d R)
{
  std::vector<Eigen::Vector3d> points;

  // tf::Quaternion quat = transform.getRotation();
  // tf::Vector3 origin = transform.getOrigin();
  // Eigen::Quaterniond quat2(quat[3], quat[0], quat[1], quat[2]);
  // Eigen::Matrix3d R2 = quat2.normalized().toRotationMatrix();
  // Eigen::Vector3d o(origin[0], origin[1], origin[2]);

  for (auto point : contour)
  {
    Eigen::Vector3d p;
    p.x() = point.x / fx_ * fabs(z_plane) * scale_it_ + xxx_;
    p.y() = point.y / fy_ * fabs(z_plane) * scale_it_ + yyy_;
    p.z() = z_plane;
    p = R * p;
    // p = R2 * p + o;
    points.push_back(p);
  }
  hole_results_.push_back(points);
}

std::vector<std::vector<Eigen::Vector3d>> HoleDetection::getHoleTargetFrame()
{
  return hole_results_;
}

std::vector<hole_detection::Hole> HoleDetection::getHolesMessage()
{
  return hole_msgs_;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> HoleDetection::getClouds()
{
  return clouds_;
}

}  // namespace hole_detection