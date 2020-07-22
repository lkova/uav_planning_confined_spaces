#include "hole_detection/HoleDetectionPackage.hpp"

namespace hole_detection
{
HoleDetectionPackage::HoleDetectionPackage(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  holeDetection_.loadAllTemplates();
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1, &HoleDetectionPackage::cloudCallback, this);
  // pose_subscriber_ = nodeHandle_.subscribe("/firefly/ground_truth/pose", 1,
  // &HoleDetectionPackage::cloudCallback, this);

  publisher_ = nodeHandle_.advertise<geometry_msgs::PointStamped>("hole_central_point", 1);

  publish_segmented_plane_ = nodeHandle_.advertise<pcl::PointCloud<pcl::PointXYZ>>("segmented_plane", 1000);
  marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  publisher_holes = nodeHandle_.advertise<hole_detection::Hole>("hole", 1);
}

HoleDetectionPackage::~HoleDetectionPackage()
{
}

bool HoleDetectionPackage::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_))
    return false;
  if (!nodeHandle_.getParam("camera_frame_topic", cameraFrameTopic_))
    return false;
  if (!nodeHandle_.getParam("debug_vis", holeDetection_.debug_))
    holeDetection_.debug_ = false;
  // if (!nodeHandle_.getParam("front", holeDetection_.is_front_))
  //     return false;
  if (!nodeHandle_.getParam("mynteye", is_mynteye_))
    is_mynteye_ = false;
  if (!nodeHandle_.getParam("fx", holeDetection_.fx_))
    holeDetection_.fx_ = 205.46;
  if (!nodeHandle_.getParam("fy", holeDetection_.fy_))
    holeDetection_.fy_ = 205.46;
  if (!nodeHandle_.getParam("cx", holeDetection_.cx_))
    holeDetection_.cx_ = 320.5;
  if (!nodeHandle_.getParam("cy", holeDetection_.cy_))
    holeDetection_.cy_ = 240.5;
  if (!nodeHandle_.getParam("target_frame", target_frame_id_))
    target_frame_id_ = "world";

  // if (!nodeHandle_.getParam("obrisi", holeDetection_.obrisi_))
  //   holeDetection_.obrisi_ = 0;

  if (!nodeHandle_.getParam("manhole_path", holeDetection_.manhole_path_))
    return false;
  if (!nodeHandle_.getParam("big_hole_path", holeDetection_.big_hole_path_))
    return false;

  holeDetection_.ismynteye_ = is_mynteye_;
  return true;
}

void HoleDetectionPackage::poseCallback(const geometry_msgs::PointStampedPtr &msg)
{
  drone_pose_ = msg;
}

void HoleDetectionPackage::getTransform(tf::StampedTransform &transform, bool &found)
{
  static tf::TransformListener listener;
  try
  {
    geometry_msgs::PointStamped p_world;
    listener.lookupTransform(target_frame_id_, cameraFrameTopic_, ros::Time(0), transform);
    found = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void HoleDetectionPackage::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  pcl::PCLPointCloud2 cloud_pcl;
  pcl_conversions::toPCL(*cloud_msg, cloud_pcl);

  if (!is_mynteye_)
  {
    tf::StampedTransform transform;
    bool is_trans = false;
    getTransform(transform, is_trans);

    if (is_trans)
    {
      holeDetection_.setCloud(cloud_pcl);
      // publish_segmented_plane_.publish(holeDetection_.cloud_c);

      std::vector<hole_detection::Hole> holes = holeDetection_.getHolesMessage();
      std::vector<std::vector<Eigen::Vector3d>> holes_full = holeDetection_.getHoleTargetFrame();
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = holeDetection_.getClouds();

      tf::Quaternion quat = transform.getRotation();
      tf::Vector3 origin = transform.getOrigin();
      Eigen::Quaterniond quat2(quat[3], quat[0], quat[1], quat[2]);
      Eigen::Matrix3d R = quat2.normalized().toRotationMatrix();
      Eigen::Vector3d o(origin[0], origin[1], origin[2]);

      for (int i = 0; i < holes.size(); i++)
      {
        holes[i].header.frame_id = target_frame_id_;

        Eigen::Vector3d point(holes[i].point.x, holes[i].point.y, holes[i].point.z);
        point = R * point + o;

        Eigen::Vector3d plane_normal(holes[i].normal.x, holes[i].normal.y, holes[i].normal.z);
        plane_normal = R * plane_normal;

        holes[i].point.x = point.x();
        holes[i].point.y = point.y();
        holes[i].point.z = point.z();
        holes[i].normal.x = plane_normal.x();
        holes[i].normal.y = plane_normal.y();
        holes[i].normal.z = plane_normal.z();

        publisher_holes.publish(holes[i]);

        for (int j = 0; j < holes_full[i].size(); j++)
        {
          holes_full[i][j] = R * holes_full[i][j] + o;
        }
      }
      if (clouds.size() > 0)
      {
        clouds[0]->header.frame_id = cameraFrameTopic_;
        publish_segmented_plane_.publish(clouds[0]);
      }
      publishHolesTF(holes);
      visualizeHoles(holes_full, holes);
    }
  }
  else
  {
    holeDetection_.setCloud(cloud_pcl);
    std::cout << "cloud " << holeDetection_.cloud_c->size() << std::endl;
    // publish_segmented_plane_.publish(holeDetection_.cloud_c);

    std::vector<hole_detection::Hole> holes = holeDetection_.getHolesMessage();
    std::vector<std::vector<Eigen::Vector3d>> holes_full = holeDetection_.getHoleTargetFrame();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds = holeDetection_.getClouds();

    if (clouds.size() > 0)
    {
      clouds[0]->header.frame_id = target_frame_id_;
      publish_segmented_plane_.publish(clouds[0]);
    }

    for (int i = 0; i < holes.size(); i++)
    {
      publisher_holes.publish(holes[i]);
    }
    publishHolesTF(holes);
    visualizeHoles(holes_full, holes);
  }
}

void HoleDetectionPackage::visualizeHoles(std::vector<std::vector<Eigen::Vector3d>> holes,
                                          std::vector<hole_detection::Hole> msgs)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = target_frame_id_;
  marker.header.stamp = ros::Time();
  marker.ns = "hole";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.1);
  // marker.pose.position.x = 1;
  // marker.pose.position.y = 1;
  // marker.pose.position.z = 1;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.scale.y = 0.04;
  // marker.scale.z = 0.1;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  int a = 1;
  for (int i = 0; i < holes.size(); i++)
  {
    for (auto point : holes[i])
    {
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      marker.points.push_back(p);
    }
    // geometry_msgs::Point p;
    // p.x = msgs[i].point.x;
    // p.x = msgs[i].point.y;
    // p.x = msgs[i].point.z;
    // marker.points.push_back(p);
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = target_frame_id_;
    arrow.header.stamp = ros::Time();
    arrow.ns = "arrow" + std::to_string(a);
    arrow.id = a++;
    arrow.type = visualization_msgs::Marker::LINE_STRIP;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 0.1;
    arrow.scale.y = 0.1;
    // marker.scale.z = 0.1;
    arrow.color.a = 1.0;  // Don't forget to set the alpha!
    arrow.color.r = 1.0;
    arrow.color.g = 0.0;
    arrow.color.b = 0.0;

    geometry_msgs::Point tmp_p;
    tmp_p.x = msgs[i].point.x;
    tmp_p.y = msgs[i].point.y;
    tmp_p.z = msgs[i].point.z;

    arrow.points.push_back(tmp_p);
    tmp_p.x += msgs[i].normal.x;
    tmp_p.y += msgs[i].normal.y;
    tmp_p.z += msgs[i].normal.z;
    arrow.points.push_back(tmp_p);
    // marker_pub.publish(arrow);
  }

  marker_pub.publish(marker);
}

void HoleDetectionPackage::publishHolesTF(std::vector<hole_detection::Hole> holes)
{
  static tf::TransformBroadcaster br;
  tf::StampedTransform transform;

  for (int i = 0; i < holes.size(); i++)
  {
    transform.setOrigin(tf::Vector3(holes[i].point.x, holes[i].point.y, holes[i].point.z));
    tf::Quaternion q(tf::Vector3(holes[i].normal.x, holes[i].normal.y, holes[i].normal.z),
                     holes[i].angle / 180.0 * M_PI);
    transform.setRotation(q);
    transform.stamp_ = ros::Time(0);
    transform.frame_id_ = cameraFrameTopic_;
    transform.child_frame_id_ = "hole_" + std::to_string(i);
    br.sendTransform(transform);
  }
}

}  // namespace hole_detection