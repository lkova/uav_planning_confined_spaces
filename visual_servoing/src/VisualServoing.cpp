#include "visual_servoing/VisualServoing.hpp"

namespace visual_servoing
{
VisualServoing::VisualServoing()
{
  int nfeatures = 1000;
  detector_ = cv::ORB::create(nfeatures);
  transformation_ = Eigen::Matrix4f();
  transformation_.setIdentity();
}

VisualServoing::~VisualServoing()
{
}

void VisualServoing::setImage(cv::Mat &image, cv::Mat &depth_image, bool isref)
{
  if (params.iscutoff && image.cols > 300)
  {
    // image = image(cv::Rect(cv::Point(0, 200), cv::Point(image.cols, image.rows)));
    int widh = image.cols / 4;
    int heih = image.rows / 4;
    image = image(cv::Rect(cv::Point(0 + widh, 0 + heih), cv::Point(image.cols - widh, image.rows - heih)));
  }

  if (isref)
  {
    reference_image_.image = image.clone();
    reference_image_.depth_image = depth_image.clone();
    findFeatures(reference_image_);
  }
  else
  {
    image_.image = image.clone();
    image_.depth_image = depth_image.clone();
    findFeatures(image_);
  }
}

void VisualServoing::execute()
{
  if (params.match_type == 0)
    matchImages();
  else
    matchImages2();
}

void VisualServoing::findFeatures(ImageFeatures &image)
{
  detector_->detectAndCompute(image.image, cv::Mat(), image.keypoints, image.descriptors);
}

void VisualServoing::matchImages()
{
  std::vector<cv::DMatch> matches;
  cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

  matcher->match(reference_image_.descriptors, image_.descriptors, matches, cv::Mat());

  std::sort(matches.begin(), matches.end());

  std::vector<cv::DMatch> good_matches;

  float th = 30;

  std::vector<cv::KeyPoint> matched1, matched2;
  int good_cout = 0;
  for (int i = 0; i < matches.size(); i++)
  {
    float th2 = 0.0;
    if (matches.size() > 1)
    {
      th2 = matches[1].distance * 2;
    }
    if (matches[i].distance < std::max(th2, th))
    {
      good_matches.push_back(matches[i]);
      matched1.push_back(reference_image_.keypoints[matches[i].queryIdx]);
      matched2.push_back(image_.keypoints[matches[i].trainIdx]);

      good_cout++;
      if (good_cout > (matches.size() / 2))
        break;
    }
    else
    {
      break;
    }
  }

  if (good_matches.size() >= 4)
  {
    std::vector<cv::DMatch> good_matches_filtered, good_matches_filtered2;
    std::vector<cv::KeyPoint> inliers1, inliers2;
    ransacTest(good_matches, matched1, matched2, good_matches_filtered, inliers1, inliers2, 0.5, 0.99, 0);

    std::cout << "Raw matches " << matches.size() << std::endl;
    std::cout << "Good matches " << good_matches.size() << std::endl;
    std::cout << "Inlier matches " << inliers1.size() << " " << good_matches_filtered.size() << std::endl;

    if (params.debug)
    {
      cv::Mat img_matches;

      cv::drawMatches(reference_image_.image, reference_image_.keypoints, image_.image, image_.keypoints,
                      good_matches_filtered, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                      cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

      cv::resize(img_matches, img_matches, cv::Size(), 0.5, 0.5);
      cv::imshow("Good matches", img_matches);

      cv::waitKey(5);
    }

    std::vector<Eigen::Vector3d> ref_vector, dest_vector;
    std::vector<std::vector<Eigen::Vector3d>> ref_addition, dest_addition;
    double sums = 0.0;

    int img_width = reference_image_.image.cols;
    int img_height = reference_image_.image.rows;

    for (int i = 0; i < inliers2.size(); i++)
    {
      cv::Point2f point_ref = inliers1[i].pt;

      cv::Point2f point_img = inliers2[i].pt;
      if (params.iscutoff)
      {
        point_img.y += 200;
        point_ref.y += 200;
      }

      // check if point is in good space
      int delta = params.patch_size + 1;
      if (point_ref.x < delta || point_ref.x > img_width - delta || point_ref.y < delta ||
          point_ref.y > img_height - delta || point_img.x < delta || point_img.x > img_width - delta ||
          point_img.y < delta || point_img.y > img_height - delta)
      {
        continue;
      }
      float depth_img = image_.depth_image.at<float>((int)point_img.y, (int)point_img.x);

      float depth_ref = reference_image_.depth_image.at<float>((int)point_ref.y, (int)point_ref.x);

      if (!isnan(depth_img) && !isnan(depth_ref) && fabs(depth_ref) > 0.0001 && fabs(depth_ref) < 3)
      {
        cv::Point3f point3d_img;
        point3d_img.z = depth_img;
        projectTo3d(point_img, point3d_img);

        cv::Point3f point3d_ref;
        point3d_ref.z = depth_ref;
        projectTo3d(point_ref, point3d_ref);

        // std::cout << "point_img x y z " << point3d_img.x << " " << point3d_img.y << " " << point3d_img.z <<
        // std::endl; std::cout << "point_ref x y z " << point3d_ref.x << " " << point3d_ref.y << " " << point3d_ref.z
        // << std::endl; std::cout << "diff " << point3d_ref.x - point3d_img.x << " " << point3d_ref.y - point3d_img.y
        // << " "
        //           << point3d_ref.z - point3d_img.z << std::endl;

        double diff = sqrt(pow(point3d_ref.x - point3d_img.x, 2) + pow(point3d_ref.y - point3d_img.y, 2) +
                           pow(point3d_ref.z - point3d_img.z, 2));
        sums += diff;

        Eigen::Vector3d pref(point3d_ref.x, point3d_ref.y, point3d_ref.z);
        Eigen::Vector3d ptar(point3d_img.x, point3d_img.y, point3d_img.z);
        bool dx = fabs(point3d_ref.x - point3d_img.x) < 2.0;
        bool dy = fabs(point3d_ref.y - point3d_img.y) < 2.0;
        bool dz = fabs(point3d_ref.z - point3d_img.z) < 2.0;
        if (dx && dy && dz)
        {
          ref_vector.push_back(pref);
          dest_vector.push_back(ptar);

          std::vector<Eigen::Vector3d> temp_ref, temp_dest;
          // KERNEL START
          for (int ki = -params.patch_size; ki <= params.patch_size; ki++)
          {
            for (int kj = -params.patch_size; kj <= params.patch_size; kj++)
            {
              if (ki != 0 || kj != 0)
              {
                cv::Point2f point_ref2(point_ref.x + kj, point_ref.y + ki);
                cv::Point2f point_img2(point_img.x + kj, point_img.y + ki);

                float depth_img2 = image_.depth_image.at<float>((int)point_img2.y, (int)point_img2.x);

                float depth_ref2 = reference_image_.depth_image.at<float>((int)point_ref2.y, (int)point_ref2.x);

                if (fabs(depth_img - depth_img2) < 0.2 && fabs(depth_ref - depth_ref2) < 0.2)
                {
                  cv::Point3f point3d_img2;
                  point3d_img2.z = depth_img2;
                  projectTo3d(point_img2, point3d_img2);

                  cv::Point3f point3d_ref2;
                  point3d_ref2.z = depth_ref2;
                  projectTo3d(point_ref2, point3d_ref2);

                  Eigen::Vector3d pref2(point3d_ref2.x, point3d_ref2.y, point3d_ref2.z);
                  Eigen::Vector3d ptar2(point3d_img2.x, point3d_img2.y, point3d_img2.z);

                  temp_ref.push_back(pref2);
                  temp_dest.push_back(ptar2);
                }
              }
            }
          }
          ref_addition.push_back(temp_ref);
          dest_addition.push_back(temp_dest);
          // KERNEL END
        }
      }
    }

    const int N = ref_vector.size();
    Eigen::Matrix3Xf ref_points(3, N), target_points(3, N);

    std::vector<Eigen::Matrix3Xf> ref_matrix_vector, dest_matrix_vector;

    for (int i = 0; i < N; i++)
    {
      ref_points(0, i) = ref_vector[i].x();
      ref_points(1, i) = ref_vector[i].y();
      ref_points(2, i) = ref_vector[i].z();

      target_points(0, i) = dest_vector[i].x();
      target_points(1, i) = dest_vector[i].y();
      target_points(2, i) = dest_vector[i].z();

      const int M = ref_addition[i].size();

      Eigen::Matrix3Xf ref_matrix(3, M), target_matrix(3, M);

      for (int j = 0; j < M; j++)
      {
        ref_matrix(0, j) = ref_addition[i][j].x();
        ref_matrix(1, j) = ref_addition[i][j].y();
        ref_matrix(2, j) = ref_addition[i][j].z();
        target_matrix(0, j) = dest_addition[i][j].x();
        target_matrix(1, j) = dest_addition[i][j].y();
        target_matrix(2, j) = dest_addition[i][j].z();
      }
      ref_matrix_vector.push_back(ref_matrix);
      dest_matrix_vector.push_back(target_matrix);
    }

    if (N > 0)
      // estimateRigidTransform(ref_points, target_points);
      // ransac(ref_points, target_points);
      ransacPatch(ref_points, target_points, ref_matrix_vector, dest_matrix_vector);
  }
}

void VisualServoing::matchImages2()
{
  if (image_.descriptors.type() != CV_32F)
  {
    reference_image_.descriptors.convertTo(reference_image_.descriptors, CV_32F);
    image_.descriptors.convertTo(image_.descriptors, CV_32F);
  }

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
  std::vector<std::vector<cv::DMatch>> matches_raw;

  matcher->knnMatch(reference_image_.descriptors, image_.descriptors, matches_raw, 2);

  std::vector<cv::Point2f> points1, points2;
  std::vector<cv::KeyPoint> kp1, kp2;
  std::vector<cv::DMatch> matched;
  std::vector<cv::DMatch> good_matches;
  float threshold = 0.8;

  for (int i = 0; i < matches_raw.size(); i++)
  {
    if (matches_raw[i][0].distance < threshold * matches_raw[i][1].distance)
    {
      good_matches.push_back(matches_raw[i][0]);
      points1.push_back(reference_image_.keypoints[matches_raw[i][0].queryIdx].pt);
      points2.push_back(image_.keypoints[matches_raw[i][0].trainIdx].pt);
    }
  }

  // for (int i = 0; i < matches_raw21.size(); i++) {
  //     if (matches_raw21[i][0].distance < threshold *
  //     matches_raw21[i][1].distance) {
  //         good_matches2.push_back(matches_raw21[i][0]);
  //     }
  // }

  // // Symmetric Test
  // std::vector<cv::DMatch> better_matches;
  // for (int i = 0; i < good_matches1.size(); i++) {
  //     for (int j = 0; j < good_matches2.size(); j++) {
  //         if (good_matches1[i].queryIdx == good_matches2[j].trainIdx &&
  //         good_matches2[j].queryIdx == good_matches1[i].trainIdx) {
  //             better_matches.push_back(cv::DMatch(good_matches1[i].queryIdx,
  //             good_matches1[i].trainIdx, good_matches1[i].distance)); break;
  //         }
  //     }
  // }

  std::vector<uchar> inliers(good_matches.size(), 0);
  std::vector<cv::KeyPoint> inliers1, inliers2;
  std::vector<cv::DMatch> inlier_matches;
  cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 2.5, 0.99, inliers);

  for (uint i = 0; i < good_matches.size(); i++)
  {
    if (inliers[i])
    {
      inliers1.push_back(kp1[i]);
      inliers2.push_back(kp2[i]);
      inlier_matches.push_back(
          cv::DMatch(good_matches[i].queryIdx, good_matches[i].trainIdx, good_matches[i].distance));
    }
  }

  // std::cout << "Raw matches " << matches_raw.size() << std::endl;
  // std::cout << "Good matches " << good_matches.size() << std::endl;
  // std::cout << "Inlier matches " << inlier_matches.size() << std::endl;

  if (params.debug)
  {
    cv::Mat img_matches;

    cv::drawMatches(reference_image_.image, reference_image_.keypoints, image_.image, image_.keypoints, inlier_matches,
                    img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    cv::imshow("Good matches", img_matches);
    cv::waitKey(5);
  }

  std::vector<Eigen::Vector3d> ref_vector, dest_vector;

  for (int i = 0; i < inliers2.size(); i++)
  {
    cv::Point2f point_ref = inliers1[i].pt;

    cv::Point2f point_img = inliers2[i].pt;

    if (params.iscutoff)
    {
      point_img.y += 200;
      point_ref.y += 200;
    }
    float depth_img = image_.depth_image.at<float>((int)point_img.y, (int)point_img.x);

    float depth_ref = reference_image_.depth_image.at<float>((int)point_ref.y, (int)point_ref.x);

    if (!isnan(depth_img) && !isnan(depth_ref))
    {
      cv::Point3f point3d_img;
      point3d_img.z = depth_img;
      projectTo3d(point_img, point3d_img);

      cv::Point3f point3d_ref;
      point3d_ref.z = depth_ref;
      projectTo3d(point_ref, point3d_ref);

      // std::cout << "point_img x y z " << point3d_img.x << " " << point3d_img.y << " " << point3d_img.z << std::endl;
      // std::cout << "point_ref x y z " << point3d_ref.x << " " << point3d_ref.y << " " << point3d_ref.z << std::endl;
      // std::cout << "diff " << point3d_ref.x - point3d_img.x << " " << point3d_ref.y - point3d_img.y << " "
      //           << point3d_ref.z - point3d_img.z << std::endl;

      Eigen::Vector3d pref(point3d_ref.x, point3d_ref.y, point3d_ref.z);
      Eigen::Vector3d ptar(point3d_img.x, point3d_img.y, point3d_img.z);

      ref_vector.push_back(pref);
      dest_vector.push_back(ptar);
    }
  }
  const int N = ref_vector.size();
  Eigen::Matrix3Xf ref_points(3, N), target_points(3, N);

  for (int i = 0; i < N; i++)
  {
    ref_points(0, i) = ref_vector[i].x();
    ref_points(1, i) = ref_vector[i].y();
    ref_points(2, i) = ref_vector[i].z();

    target_points(0, i) = dest_vector[i].x();
    target_points(1, i) = dest_vector[i].y();
    target_points(2, i) = dest_vector[i].z();
  }
  if (N > 0)
    // estimateRigidTransform(ref_points, target_points);
    ransac(ref_points, target_points);
}

void VisualServoing::ransacTest(const std::vector<cv::DMatch> matches, const std::vector<cv::KeyPoint> &matched1,
                                const std::vector<cv::KeyPoint> &matched2, std::vector<cv::DMatch> &goodMatches,
                                std::vector<cv::KeyPoint> &inliers1, std::vector<cv::KeyPoint> &inliers2,
                                double distance, double confidence, double minInlierRatio)
{
  std::vector<cv::Point2f> points1, points2;

  for (int i = 0; i < matched1.size(); i++)
  {
    points1.push_back(matched1[i].pt);
    points2.push_back(matched2[i].pt);
  }

  std::vector<uchar> inliers(points1.size(), 0);
  // cv::Mat fundemental = cv::findFundamentalMat(points1, points2, inliers,
  // cv::FM_RANSAC, distance, confidence);
  // std::vector<uchar>::const_iterator itIn = inliers.begin();
  // std::vector<cv::DMatch>::const_iterator itM = matches.begin();

  // for (; itIn != inliers.end(); ++itIn, ++itM) {
  //     if (*itIn) {  // it is a valid match
  //         goodMatches.push_back(*itM);
  //     }
  // }
  cv::Mat inlier_mask, homography;

  homography = findHomography(points1, points2, cv::RANSAC, 1.5f, inliers);

  for (uint i = 0; i < points1.size(); i++)
  {
    if (inliers[i])
    {
      int new_i = static_cast<int>(inliers1.size());
      inliers1.push_back(matched1[i]);
      inliers2.push_back(matched2[i]);
      goodMatches.push_back(cv::DMatch(matches[i].queryIdx, matches[i].trainIdx, matches[i].distance));
    }
  }
}

Eigen::Matrix4f VisualServoing::estimateRigidTransform(Eigen::Matrix3Xf &points_reference,
                                                       Eigen::Matrix3Xf &points_target)
{
  Eigen::Vector3f centroid_reference = points_reference.rowwise().mean();
  Eigen::Vector3f centroid_target = points_target.rowwise().mean();

  points_reference = points_reference.colwise() - centroid_reference;
  points_target = points_target.colwise() - centroid_target;

  Eigen::Matrix3Xf H = points_reference * points_target.transpose();

  Eigen::JacobiSVD<Eigen::Matrix3Xf> svd =
      H.jacobiSvd(Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);

  Eigen::Matrix3f U = svd.matrixU();
  Eigen::MatrixXf V = svd.matrixV();
  Eigen::Matrix3Xf S = svd.singularValues();
  Eigen::Matrix3f R = V * U.transpose();

  // reflection
  if (R.determinant() < 0.0f)
  {
    V.col(2) *= -1.0f;
    R = V * U.transpose();
  }

  Eigen::Vector3f t = centroid_target - R * centroid_reference;
  // std::cout << "R = \n" << R << std::endl;
  // std::cout << "t = " << t << std::endl;
  // std::cout << "S = " << S << std::endl;

  Eigen::Matrix4f trans;
  trans.setIdentity();
  trans.block<3, 3>(0, 0) = R;
  trans.block<3, 1>(0, 3) = t;

  // std::cout << "trans " << trans << std::endl;
  return trans;
}

void VisualServoing::projectTo3d(const cv::Point2f &point2d, cv::Point3f &point3d)
{
  point3d.x = (point2d.x - params.cx) * point3d.z / params.fx;
  point3d.y = (point2d.y - params.cy) * point3d.z / params.fy;
}

void VisualServoing::visualize()
{
  cv::Mat img_matches;

  cv::drawMatches(reference_image_.image, reference_image_.keypoints, image_.image, image_.keypoints, good_matches_,
                  img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("Good matches", img_matches);

  cv::waitKey(30);
}

void VisualServoing::customDrawMatches(const cv::Mat &im1, const std::vector<cv::Point2f> &pts_1, const cv::Mat &im2,
                                       const std::vector<cv::Point2f> &pts_2, cv::Mat &outImage, const std::string msg,
                                       std::vector<uchar> status, bool enable_lines, bool enable_points,
                                       bool enable_test)
{
  assert(pts_1.size() == pts_2.size());
  assert(im1.rows == im2.rows);
  assert(im2.cols == im2.cols);
  assert(im1.channels() == im2.channels());

  cv::Mat row_im;
  cv::hconcat(im1, im2, row_im);

  if (row_im.channels() == 3)
    outImage = row_im.clone();
  else
    cvtColor(row_im, outImage, CV_GRAY2RGB);

  // loop  over points
  for (int i = 0; i < pts_1.size(); i++)
  {
    cv::Point2f p1 = pts_1[i];
    cv::Point2f p2 = pts_2[i];

    if (enable_points)
    {
      cv::circle(outImage, p1, 4, cv::Scalar(0, 0, 255), -1);
      cv::circle(outImage, p2 + cv::Point2f(im1.cols, 0), 4, cv::Scalar(0, 0, 255), -1);
    }

    if (enable_test)
    {
      cv::putText(outImage, std::to_string(i).c_str(), p1, cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0, 0, 255));
      cv::putText(outImage, std::to_string(i).c_str(), p2 + cv::Point2f(im1.cols, 0), cv::FONT_HERSHEY_COMPLEX_SMALL,
                  .5, cv::Scalar(0, 0, 255));
    }

    if (enable_lines)
    {
      if (status.size() > 0)  // if status is present
      {
        if (status[i] > 0)
        {
          cv::line(outImage, p1, p2 + cv::Point2f(im1.cols, 0), cv::Scalar(0, 255, 0));
        }
        else
        {
          cv::line(outImage, p1, p2 + cv::Point2f(im1.cols, 0), cv::Scalar(0, 0, 255));
        }
      }
      else  // no status. Then make all lines blue
      {
        cv::line(outImage, p1, p2 + cv::Point2f(im1.cols, 0), cv::Scalar(255, 0, 0));
      }
    }
  }

  if (msg.length() > 0)
  {
    cv::putText(outImage, msg, cv::Point(5, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(0, 0, 255));
  }
}

void VisualServoing::ransac(Eigen::Matrix3Xf &points_reference, Eigen::Matrix3Xf &points_target)
{
  int max_size = points_reference.cols();

  if (max_size < 4)
  {
    transformation_ = estimateRigidTransform(points_reference, points_target);
    return;
  }

  srand(time(NULL));

  const int n_trials = 3;

  // std::cout << max_size << std::endl;
  int n_min = 3;
  int sizze = points_reference.cols();
  int k_max = std::min(sizze, 50);
  int iteration = 0;
  float max_error = 0.05;
  int required_number = 0.7 * max_size;
  int max_inliers = 0;

  Eigen::Matrix4f best_trans;
  while (iteration < k_max)
  {
    int j = 0;

    // get all test inliers
    Eigen::Matrix3Xf ref_points(3, n_trials), target_points(3, n_trials);
    std::unordered_set<int> indices;
    while (j < n_trials)
    {
      int index = rand() % max_size;
      if (indices.find(index) == indices.end())
      {
        indices.insert(index);

        ref_points.col(j) = points_reference.col(index);
        target_points.col(j) = points_target.col(index);
        j++;
      }
    }

    // std::cout << "ref " << ref_points << std::endl;

    Eigen::Matrix4f trans = estimateRigidTransform(ref_points, target_points);

    Eigen::Matrix3f rot = trans.block<3, 3>(0, 0);
    Eigen::Vector3f t = trans.block<3, 1>(0, 3);

    Eigen::VectorXf error_array = ((points_target - rot * points_reference).colwise() - t).colwise().stableNorm();

    int n_inliers = (error_array.array() < max_error).count();
    if (n_inliers > required_number)
    {
      if (n_inliers > max_inliers)
      {
        max_inliers = n_inliers;
        best_trans = Eigen::Matrix4f(trans);
      }
    }
    iteration++;
  }
  // std::cout << "max_inliers " << max_inliers << " " << max_size << std::endl;
  // std::cout << best_trans << std::endl;
  transformation_ = best_trans;
}

void VisualServoing::ransacPatch(Eigen::Matrix3Xf &points_reference, Eigen::Matrix3Xf &points_target,
                                 std::vector<Eigen::Matrix3Xf> &reference_vector,
                                 std::vector<Eigen::Matrix3Xf> &target_vector)
{
  int max_size = points_reference.cols();
  const int n_trials = 12;
  if (max_size < n_trials)
  {
    // transformation_ = estimateRigidTransform(points_reference, points_target);

    int i = 0;
    int sizes = 0;

    for (int j = 0; j < points_reference.cols(); j++)
    {
      sizes++;
    }
    for (int k = 0; k < reference_vector.size(); k++)
    {
      sizes += reference_vector[k].cols();
    }
    const int size2 = sizes;
    Eigen::Matrix3Xf all_ref(3, size2), all_tar(3, size2);
    for (int j = 0; j < points_reference.cols(); j++)
    {
      all_ref.col(i) = points_reference.col(j);
      all_tar.col(i) = points_target.col(j);
      i++;
    }
    for (int k = 0; k < reference_vector.size(); k++)
    {
      for (int j = 0; j < reference_vector[k].cols(); j++)
      {
        all_ref.col(i) = reference_vector[k].col(j);
        all_tar.col(i) = target_vector[k].col(j);
        i++;
      }
    }
    transformation_ = estimateRigidTransform(all_ref, all_tar);
    static int broj5 = 0;
    float dd = 0.02;
    Eigen::Vector3f t1 = transformation_.block<3, 1>(0, 3);
    if (fabs(t1.x()) > dd || fabs(t1.y()) > dd || fabs(t1.z()) > dd)
    {
      broj5++;
    }
    std::cout << "broj 5 " << broj5 << std::endl;
    return;
  }

  srand(time(NULL));

  // std::cout << max_size << std::endl;
  int n_min = 3;
  int sizze = points_reference.cols();
  int k_max = std::min(sizze, 50);
  int iteration = 0;
  float max_error = 0.03;
  int required_number = 0.8 * max_size;
  int max_inliers = 0;

  Eigen::Matrix4f best_trans;

  // probably shouldn't work;
  // std::vector<Eigen::Matrix4f> best_transes;

  int max_number_new = 4;
  std::vector<int> max_inliers_all = std::vector<int>(max_number_new, 0);
  std::vector<Eigen::Matrix4f> best_transes = std::vector<Eigen::Matrix4f>(max_number_new, Eigen::Matrix4f());

  while (iteration < k_max)
  {
    int j = 0;

    // get all test inliers

    std::unordered_set<int> indices;
    int additional_size = 0;
    while (j < n_trials)
    {
      int index = rand() % max_size;
      if (indices.find(index) == indices.end())
      {
        indices.insert(index);
        additional_size += reference_vector[index].cols();
        // std::cout << "SIZEEEEEEEEEE " << reference_vector[index].cols();
        j++;
      }
    }
    j = 0;
    const int final_size = n_trials + additional_size;
    Eigen::Matrix3Xf ref_points(3, final_size), target_points(3, final_size);

    // std::cout << "final size " << final_size << std::endl;
    for (int index : indices)
    {
      ref_points.col(j) = points_reference.col(index);
      target_points.col(j) = points_target.col(index);
      j++;
      for (int k = 0; k < reference_vector[index].cols(); k++)
      {
        ref_points.col(j) = reference_vector[index].col(k);
        target_points.col(j) = target_vector[index].col(k);
        j++;
      }
    }
    // std::cout << "ref " << ref_points << std::endl;

    Eigen::Matrix4f trans = estimateRigidTransform(ref_points, target_points);

    Eigen::Matrix3f rot = trans.block<3, 3>(0, 0);
    Eigen::Vector3f t = trans.block<3, 1>(0, 3);

    Eigen::VectorXf error_array = ((points_target - rot * points_reference).colwise() - t).colwise().stableNorm();

    int n_inliers = (error_array.array() < max_error).count();
    if (n_inliers > required_number)
    {
      if (n_inliers > max_inliers)
      {
        // del from
        for (int index_best = max_number_new - 1; index_best > 0; index_best--)
        {
          best_transes[index_best] = best_transes[index_best - 1];
          max_inliers_all[index_best] = max_inliers_all[index_best - 1];
        }
        // del to
        best_transes[0] = Eigen::Matrix4f(trans);
        max_inliers_all[0] = n_inliers;

        max_inliers = n_inliers;
        best_trans = Eigen::Matrix4f(trans);
      }
    }
    iteration++;
  }
  std::cout << "max_inliers " << max_inliers << " " << max_size << std::endl;
  // std::cout << best_trans << std::endl;
  Eigen::Vector3f t1 = best_trans.block<3, 1>(0, 3);

  Eigen::Matrix4f trans_final = Eigen::Matrix4f::Zero();
  int counter = 0;
  for (int i = 0; i < max_number_new; i++)
  {
    if (max_inliers_all[i] != 0)
    {
      counter++;
      Eigen::Matrix4f temp = best_transes[i];
      // std::cout << "max_inlier i " << i << " " << max_inliers_all[i] << std::endl;
      trans_final += temp;
    }
  }
  trans_final = trans_final / counter;
  Eigen::Vector3f t2 = trans_final.block<3, 1>(0, 3);

  std::cout << "t1 " << t1 << std::endl;
  std::cout << "t2 " << t2 << std::endl;
  std::cout << "end" << std::endl;
  static int broj1 = 0;
  static int broj2 = 0;

  float dd = 0.03;
  if (fabs(t1.x()) > dd || fabs(t1.y()) > dd || fabs(t1.z()) > dd)
  {
    broj1++;
  }
  if (fabs(t2.x()) > dd || fabs(t2.y()) > dd || fabs(t2.z()) > dd)
  {
    broj2++;
  }

  std::cout << "BROOOOJ " << broj1 << " " << broj2 << " " << std::endl;
  transformation_ = trans_final;
}

Eigen::Matrix4f VisualServoing::getTransformation()
{
  Eigen::Vector3f t = transformation_.block<3, 1>(0, 3);
  float x = fabs(t.x());
  float y = fabs(t.y());
  float z = fabs(t.z());

  if (x > 2 || y > 2 || z > 2)
  {
    transformation_.setIdentity();
  }
  return transformation_;
}

}  // namespace visual_servoing
