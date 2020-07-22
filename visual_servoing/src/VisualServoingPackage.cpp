#include "visual_servoing/VisualServoingPackage.hpp"

namespace visual_servoing
{
VisualServoingPackage::VisualServoingPackage(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
{
  is_ref_image_ = false;
  iscamsub_ = false;
  if (!readParameters())
  {
  }
  // subscriber_image_ = nodeHandle_.subscribe(image_topic_, 1,
  // &VisualServoingPackage::imageCallback, this);
  subscribe_raw_image_.subscribe(nodeHandle_, image_topic_, 1);
  subscribe_depth_image_.subscribe(nodeHandle_, depth_image_topic_, 1);
  subscriber_hole_ =
      nodeHandle_.subscribe("firefly/detectionFront/hole", 1, &VisualServoingPackage::holeCallback, this);

  camera_info_sub_ = nodeHandle_.subscribe(camera_info_topic_, 1, &VisualServoingPackage::cameraInfoCallback, this);

  drone_pose_subscriber_ = nodeHandle_.subscribe(pose_topic_, 1, &VisualServoingPackage::odometryCallback, this);

  sync_.reset(new Sync(MySyncPolicy(10), subscribe_raw_image_, subscribe_depth_image_));
  // sync_->registerCallback(&VisualServoingPackage::testCallback, this);
  sync_->registerCallback(boost::bind(&VisualServoingPackage::imageCallback, this, _1, _2));
  // test =
  // nodeHandle_.subscribe("/firefly/vi_sensor/camera_depth/depth/disparity", 1,
  // &VisualServoingPackage::testCallback, this);

  publisher_transform_ = nodeHandle_.advertise<geometry_msgs::TransformStamped>("error_transform", 1);
  corrected_pose_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("next_pose", 1);
}

VisualServoingPackage::~VisualServoingPackage()
{
}

bool VisualServoingPackage::readParameters()
{
  if (!nodeHandle_.getParam("image_topic", image_topic_))
  {
    return false;
  }

  if (!nodeHandle_.getParam("depth_image_topic", depth_image_topic_))
  {
    return false;
  }

  if (!nodeHandle_.getParam("target_frame", target_frame_))
  {
  }

  // if (!nodeHandle_.getParam("fx", visual_servoing.params.fx))
  // {
  //   return false;
  // }

  // if (!nodeHandle_.getParam("fy", visual_servoing.params.fy))
  // {
  //   return false;
  // }
  // if (!nodeHandle_.getParam("cx", visual_servoing.params.cx))
  // {
  //   return false;
  // }
  // if (!nodeHandle_.getParam("cy", visual_servoing.params.cy))
  // {
  //   return false;
  // }

  if (!nodeHandle_.getParam("cutoff", visual_servoing.params.iscutoff))
  {
    visual_servoing.params.iscutoff = false;
  }

  if (!nodeHandle_.getParam("debug", visual_servoing.params.debug))
  {
    visual_servoing.params.debug = false;
  }

  if (!nodeHandle_.getParam("match_type", visual_servoing.params.match_type))
  {
    visual_servoing.params.match_type = 0;
  }

  if (!nodeHandle_.getParam("pose_topic", pose_topic_))
  {
  }

  if (!nodeHandle_.getParam("patch_size", visual_servoing.params.patch_size))
  {
  }
  if (!nodeHandle_.getParam("camera_info_topic", camera_info_topic_))
  {
  }
  if (!nodeHandle_.getParam("camera_frame", camera_frame_))
  {
  }
  if (!nodeHandle_.getParam("drone_frame", base_link_frame_))
  {
  }
  if (!nodeHandle_.getParam("simulation", is_simulation_))
  {
    is_simulation_ = false;
  }
  return true;
}

void VisualServoingPackage::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_ = *msg;
}
// void VisualServoingPackage::imageCallback(const sensor_msgs::ImageConstPtr
// &msg) {
//     cv::Mat image;
//     cv_bridge::CvImageConstPtr cv_ptr;

//     try {
//         cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
//         image = cv::Mat(cv_ptr->image);

//         if (!is_ref_image_) {
//             is_ref_image_ = true;
//             visual_servoing.setImage(image, true);
//         } else {
//             visual_servoing.setImage(image);
//             visual_servoing.execute();
//             //visual_servoing.visualize();
//         }
//     }

//     catch (cv_bridge::Exception &e) {
//         ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
//         msg->encoding.c_str());
//     }
// }

void VisualServoingPackage::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  if (!iscamsub_)
  {
    visual_servoing.params.fx = msg->K[0];
    visual_servoing.params.fy = msg->K[4];
    visual_servoing.params.cx = msg->K[2];
    visual_servoing.params.cy = msg->K[5];

    iscamsub_ = true;
    std::cout << visual_servoing.params.fx << " " << visual_servoing.params.fy << " " << visual_servoing.params.cx
              << " " << visual_servoing.params.cy << std::endl;

    bool heh = true;
    // static tf::TransformListener listener5;
    // tf::StampedTransform transform5;
    // while (heh)
    // {
    //   try
    //   {
    //     listener5.lookupTransform(camera_frame_, base_link_frame_, ros::Time(0), transform5);
    //     // listener.transformPose(base_link_frame_, p_before, p_after);
    //     std::cout << transform5.getRotation().getX() << std::endl;
    //     std::cout << transform5.getRotation().getY() << std::endl;
    //     std::cout << transform5.getRotation().getZ() << std::endl;
    //     std::cout << transform5.getRotation().getW() << std::endl;
    //     std::cout << transform5.frame_id_ << std::endl;
    //     heh = false;
    //   }
    //   catch (tf::TransformException ex)
    //   {
    //     ROS_ERROR("%s", ex.what());
    //     heh = true;
    //   }
    // }
  }
}

void VisualServoingPackage::imageCallback(const sensor_msgs::ImageConstPtr &msg,
                                          const sensor_msgs::ImageConstPtr &depth_msg)
{
  // float *depths = (float *)(&depth_msg->data[0]);

  // // Image coordinates of the center pixel
  // int u = depth_msg->width / 2;
  // int v = depth_msg->height / 2;

  // // Linear index of the center pixel
  // int centerIdx = u + depth_msg->width * v;

  // // Output the measure
  // // std::cout << "u " << u << std::endl;
  // // std::cout << "v " << v << std::endl;
  // int cc = 0;
  // for (int i = 0; i < u * v * 4 - 2; i++)
  // {
  //   ROS_INFO("Center distance : %g m", depths[i]);
  // }
  // // ROS_INFO("Center distance : %g m", depths[centerIdx]);
  // return;

  // cv::Mat depth_image;
  // cv_bridge::CvImageConstPtr cv_ptr_depth;

  if (!iscamsub_)
    return;
  auto start_time_ = std::chrono::system_clock::now();
  cv::Mat image;
  cv_bridge::CvImageConstPtr cv_ptr;

  cv::Mat depth_image;
  cv_bridge::CvImageConstPtr cv_ptr_depth;

  // depth_image = cv::Mat(cv_ptr_depth->image);
  // cv_ptr_depth->image.convertTo(depth_image, CV_32F, 0.001);

  // float depth = depth_image.at<float>(v, u);
  // std::cout << "depth " << depth << std::endl;

  try
  {
    if (is_simulation_)
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

      image = cv::Mat(cv_ptr->image);

      cv_ptr_depth = cv_bridge::toCvShare(depth_msg, "32FC1");

      depth_image = cv::Mat(cv_ptr_depth->image);
    }
    else
    {
      cv_ptr = cv_bridge::toCvShare(msg, "mono8");
      image = cv::Mat(cv_ptr->image);
      cv_ptr_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::MONO16);
      depth_image = cv::Mat(cv_ptr_depth->image);
      cv_ptr_depth->image.convertTo(depth_image, CV_32F, 0.001);
    }

    // depth_image = cv::Mat(cv_ptr_depth->image);
    // float depth = depth_image.at<float>(v, u);

    if (!is_ref_image_)
    {
      is_ref_image_ = true;
      // std::cout << "data size " << depth_msg->data.size();
      visual_servoing.setImage(image, depth_image, true);
      reference_odom_ = odom_;
    }
    else
    {
      visual_servoing.setImage(image, depth_image);
      visual_servoing.execute();
      // visual_servoing.visualize();
      std::cout << "wtff" << std::endl;
      publishTransform();

      auto end_time_ = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end_time_ - start_time_;
      std::time_t end_time = std::chrono::system_clock::to_time_t(end_time_);

      std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count()
                << "s\n";
    }
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void VisualServoingPackage::holeCallback(const hole_detection::Hole &msg)
{
}

void VisualServoingPackage::publishTransform()
{
  Eigen::Matrix4f trans = visual_servoing.getTransformation();

  // std::cout << "ref odom " << odom_.pose.pose.position.x << " " <<
  // odom_.pose.pose.position.y << " "
  //           << odom_.pose.pose.position.z << std::endl;
  Eigen::Matrix4f fixed_transformed, ah;
  fixed_transformed.setIdentity();
  ah.setIdentity();
  Eigen::Quaternionf fq;
  // fq.x() = -0.5;
  // fq.y() = 0.5;
  // fq.z() = -0.5;
  // fq.w() = 0.5;

  fq.x() = -0.5;
  fq.y() = 0.5;
  fq.z() = -0.5;
  fq.w() = -0.5;

  fixed_transformed.block<3, 3>(0, 0) = fq.toRotationMatrix();
  ah.block<3, 3>(0, 0) << 0, 1, 0, -1, 0, 0, 0, 0, 1;

  // std::cout << trans << std::endl;
  // std::cout << fixed_transformed << std::endl;
  trans.block<3, 3>(0, 0) = -trans.block<3, 3>(0, 0);
  trans.block<3, 1>(0, 3) = -trans.block<3, 1>(0, 3);
  trans = fixed_transformed.inverse() * trans;
  trans = ah.inverse() * trans;
  Eigen::Matrix3f rot = trans.block<3, 3>(0, 0);
  Eigen::Vector3f t = trans.block<3, 1>(0, 3);

  std::cout << "BEGIN COMPARISON" << std::endl;
  std::cout << -reference_odom_.pose.pose.position.x + odom_.pose.pose.position.x << " " << t.x() << std::endl;
  std::cout << -reference_odom_.pose.pose.position.y + odom_.pose.pose.position.y << " " << t.y() << std::endl;
  std::cout << -reference_odom_.pose.pose.position.z + odom_.pose.pose.position.z << " " << t.z() << std::endl;
  std::cout << "END COMPARISON" << std::endl;

  Eigen::Quaternionf q(rot);
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = base_link_frame_;
  q.normalize();

  // transform_stamped.transform.translation.x = t.x();
  // transform_stamped.transform.translation.y = t.y();
  // transform_stamped.transform.translation.z = t.z();

  transform_stamped.transform.rotation.w = q.w();
  transform_stamped.transform.rotation.x = q.x();
  transform_stamped.transform.rotation.y = q.y();
  transform_stamped.transform.rotation.z = q.z();

  transform_stamped.transform.translation.x = t.x();
  transform_stamped.transform.translation.y = t.y();
  transform_stamped.transform.translation.z = t.z();

  // transform_stamped.transform.rotation.w = reference_odom_.pose.pose.orientation.w;
  // transform_stamped.transform.rotation.x = reference_odom_.pose.pose.orientation.x;
  // transform_stamped.transform.rotation.y = reference_odom_.pose.pose.orientation.y;
  // transform_stamped.transform.rotation.z = reference_odom_.pose.pose.orientation.z;

  transform_stamped.child_frame_id = "yahu";
  // move from reference point
  std::cout << "wtf " << base_link_frame_ << std::endl;
  publisher_transform_.publish(transform_stamped);
  tf::TransformBroadcaster br;
  br.sendTransform(transform_stamped);
  // corrected_pose_.publish(p_before);
  geometry_msgs::PoseStamped pose_yes;
  pose_yes.header = transform_stamped.header;
  // pose_yes.pose.position.x = transform_stamped.transform.translation.x;
  // pose_yes.pose.position.y = transform_stamped.transform.translation.y;
  // pose_yes.pose.position.z = transform_stamped.transform.translation.z;
  // pose_yes.pose.orientation.x = transform_stamped.transform.rotation.x;
  // pose_yes.pose.orientation.y = transform_stamped.transform.rotation.y;
  // pose_yes.pose.orientation.x = transform_stamped.transform.rotation.z;
  // pose_yes.pose.orientation.w = transform_stamped.transform.rotation.w;
  pose_yes.pose.position.x = transform_stamped.transform.translation.x + reference_odom_.pose.pose.position.x;
  pose_yes.pose.position.y = transform_stamped.transform.translation.y + reference_odom_.pose.pose.position.y;
  pose_yes.pose.position.z = transform_stamped.transform.translation.z + reference_odom_.pose.pose.position.z;
  pose_yes.pose.orientation.x = reference_odom_.pose.pose.orientation.x;
  pose_yes.pose.orientation.y = reference_odom_.pose.pose.orientation.y;
  pose_yes.pose.orientation.x = reference_odom_.pose.pose.orientation.z;
  pose_yes.pose.orientation.w = reference_odom_.pose.pose.orientation.w;
  corrected_pose_.publish(pose_yes);

  // geometry_msgs::TransformStamped transform_stamped;
  // transform_stamped.header = p_after.header;

  // transform_stamped.transform.translation.x = p_after.pose.position.x;
  // transform_stamped.transform.translation.y = p_after.pose.position.y;
  // transform_stamped.transform.translation.z = p_after.pose.position.z;

  // transform_stamped.transform.rotation.w = p_after.pose.orientation.w;
  // transform_stamped.transform.rotation.x = p_after.pose.orientation.x;
  // transform_stamped.transform.rotation.y = p_after.pose.orientation.y;
  // transform_stamped.transform.rotation.z = p_after.pose.orientation.z;

  // publisher_transform_.publish(transform_stamped);

  // // new addition for target pose

  // geometry_msgs::PoseStamped pose;
  // pose.header.frame_id = "world";
  // pose.header.stamp = ros::Time::now();

  // pose.pose.position.x = reference_odom_.pose.pose.position.x - transform_stamped.transform.translation.x;
  // pose.pose.position.y = reference_odom_.pose.pose.position.y - transform_stamped.transform.translation.y;
  // pose.pose.position.z = reference_odom_.pose.pose.position.z - transform_stamped.transform.translation.z;

  // pose.pose.orientation = reference_odom_.pose.pose.orientation;
  // corrected_pose_.publish(pose);
}
}  // namespace visual_servoing