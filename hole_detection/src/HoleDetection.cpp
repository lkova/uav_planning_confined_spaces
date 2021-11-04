#include "hole_detection/HoleDetection.hpp"

namespace hole_detection
{
  HoleDetection::HoleDetection()
  {
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
    cloud_ = loadCloudData(cloud_msg);
    cloud_filtered_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_p_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_c = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pub = false;

    // filter point cloud
    if (!ismynteye_)
      filterCloud(cloud_);

    // process point cloud
    if (cloud_->size() > 0)
    {
      pcl::copyPointCloud(*cloud_, *cloud_c);
      multiTest();
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr HoleDetection::loadCloudData(const pcl::PCLPointCloud2 &cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_msg, *cloud);

    return cloud;
  }

  // CHANGE DATA LOADING TO YAML OR BINARY FILE
  void HoleDetection::loadTemplate()
  {
    hole_circle_.tempx = Eigen::VectorXd::Zero(279);
    hole_circle_.tempy = Eigen::VectorXd::Zero(279);

    std::ifstream myfile(manhole_path_);

    int i = 0;
    if (myfile.is_open())
    {
      int a, b;
      while (myfile >> a >> b)
      {
        template_contour_.push_back(cv::Point(a, b));
        hole_circle_.tempx[i] = a;
        hole_circle_.tempy[i] = b;
        i++;
      }
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
        hole_big_.tempx[i] = a;
        hole_big_.tempy[i] = b;
        i++;
      }
      myfile.close();
      hole_big_.scale_x = 1.3;

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
    // filter the cloud based on distance
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 4.5);
    pass.filter(*cloud);

    //downsample point cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.03f, 0.03f, 0.03f);
    sor.filter(*cloud);

    if (cloud->size() > 0)
    {
      pcl::copyPointCloud(*cloud, *cloud_c);
    }
  }

  void HoleDetection::multiTest()
  {
    std::vector<Plane> planes;
    multiPlaneSegmentation(cloud_, planes);

    for (Plane plane : planes)
    {

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

      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*cloud_p_, *transformed_cloud, transform_2);
      cloud_p_ = transformed_cloud;

      // save plane and its rotation
      plane.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

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
    // hole_type = 0 -> manhole
    // hole_type = 1 -> big hole | passage
    int hole_type = -1;
    int default_kernel = 3;

    // morph operations
    cv::Mat element_default = getStructuringElement(cv::MORPH_RECT, cv::Size(default_kernel, default_kernel));
    cv::dilate(image, image, element_default, cv::Point(-1, -1), 7);
    cv::erode(image, image, element_default, cv::Point(-1, -1), 7);

    cv::Mat imgCanny;
    cv::Canny(image,    // input image
              imgCanny, // output image
              30,       // low threshold
              200);     // high threshold

    cv::findContours(imgCanny, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1);

    cv::Scalar color(255, 0, 0);

    if (debug_)
    {
      cv::imshow("imgCanny", imgCanny);
      cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);
      cv::imshow("image", image);
      cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
      cv::waitKey(2);
    }

    if (contours.size() > 0)
    {
      double min = -1;
      int bci = -1;
      std::vector<std::vector<cv::Point>> cs2;
      std::vector<cv::Point> cs;

      for (int i = 0; i < contours.size(); i++)
      {
        double area = cv::contourArea(contours[i]);
        cv::Moments m = cv::moments(contours[i]);
        double cx = m.m10 / m.m00;
        double cy = m.m01 / m.m00;

        if (!isnan(cx) && !isnan(cy))
        {
          std::vector<cv::Point> curve;
          double correct_rate1 = 0.0, correct_rate2 = 0.0;

          if (area > 200)
          {
            correct_rate1 = matchShapes(contours[i], hole_circle_, z_plane);

            if (correct_rate1 <= 0.70)
            {
              correct_rate2 = matchShapes(contours[i], hole_big_, z_plane);

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
        // publish found hole
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

        if (is_point_updated)
        {
          hole_local_coordinates_.x = pos.x;
          hole_local_coordinates_.y = pos.y;
          hole_local_coordinates_.z = pos.z;

          publishHole(tran, angle, hole_type, plane_normal);

          if (hole_type == 0)
          {
            ROS_INFO("Found manhole");
            ROS_INFO("valid point %lf %lf %lf", pos.x, pos.y, pos.z);
          }
          else
          {
            ROS_INFO("Found passage");
            ROS_INFO("valid point %lf %lf %lf", pos.x, pos.y, pos.z);
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

        holeToTargetFrame(contours[bci], z_plane, rotation.inverse());

        if (debug_)
        {
          cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
          cv::circle(dst, cv::Point(cx, cy), 2, cv::Scalar(0, 0, 255));
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
          cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
          // Show windows
          cv::imshow("imgOriginal", dst);
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

  void HoleDetection::createImage(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Mat &image, double z_plane)
  {
    const std::lock_guard<std::mutex> lock(g_i_mutex);
    // change to global variables

    double resolution = 1 / fx_;

    float xmin = VTK_FLOAT_MAX;
    float xmax = VTK_FLOAT_MIN;
    float ymin = VTK_FLOAT_MAX;
    float ymax = VTK_FLOAT_MIN;
    float zmin = VTK_FLOAT_MAX;
    float zmax = VTK_FLOAT_MIN;

    defined_ = false;
    if (cloud->points.size() > 50)
    {
      for (pcl::PointXYZ point : cloud->points)
      {
        xmin = std::min(xmin, (float)point.x);
        xmax = std::max(xmax, (float)point.x);
        ymin = std::min(ymin, (float)point.y);
        ymax = std::max(ymax, (float)point.y);
        zmin = std::min(zmin, (float)point.z);
        zmax = std::max(zmax, (float)point.z);
      }

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

      cv::Mat image2(height, width, CV_8U, cv::Scalar(0));
      for (pcl::PointXYZ point : cloud->points)
      {
        cv::Point uv_rect = projectCloudTo2d(point, xmin, ymin, resolution, z_plane);
        image2.at<uchar>(uv_rect.y, uv_rect.x) = 255;

        if (uv_rect.y > height - 1 || uv_rect.y < 0 || uv_rect.x > width - 1 || uv_rect.x < 0)
        {
          std::cout << "h w out of bounds " << height << " " << width << " " << uv_rect << std::endl;
        }
      }

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

            if (diff < min_error && diff < error)
            {
              min_error = diff;
              min_index = i;
            }
          }

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
    return static_cast<double>(best_good) / static_cast<double>(contour.size());
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

} // namespace hole_detection