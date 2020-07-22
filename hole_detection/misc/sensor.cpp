#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef sensor_msgs::CameraInfo CamInfo;

double centre_x;
double centre_y;
double focal_x;
double focal_y;
int cam_height;
int cam_width;
bool published = false;
CamInfo info;
sensor_msgs::ImagePtr output_image;
image_transport::Publisher publish_result;

void callback(const PointCloud::ConstPtr& msg)
{
  if (published == true)
  {
    cv::Mat cv_image = cv::Mat(cam_height, cam_width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
    for (int i = 0; i < msg->points.size(); i++)
    {
      if (msg->points[i].z == msg->points[i].z)
      {
        float z = msg->points[i].z * 1000.0;
        float u = (msg->points[i].x * 1000.0 * focal_x) / z;
        float v = (msg->points[i].y * 1000.0 * focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

        if (pixel_pos_x > (cam_width - 1))
        {
          pixel_pos_x = cam_width - 1;
        }
        if (pixel_pos_y > (cam_height - 1))
        {
          pixel_pos_y = cam_height - 1;
        }
        cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
        float b = cv_image.at<float>(151, 212);
        ROS_INFO("zasd %f ", b);
      }
    }
    // ROS_INFO("centre_x = %lf, centre_y = %lf, focal_x = %lf, focal_y = %lf = cam_height = %d, cam_width %d",
    // centre_x,
    //         centre_y, focal_x, focal_y, cam_height, cam_width);

    cv_image.convertTo(cv_image, CV_16UC1);

    output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
    output_image->header = info.header;
    output_image->header.stamp = info.header.stamp;
    publish_result.publish(output_image);
    // publish_cam_info.publish(info);
  }
}

void camera_callback(const CamInfo::ConstPtr& cam_info)
{
  if (published == false)
  {
    centre_x = cam_info->K[2];
    centre_y = cam_info->K[5];
    focal_x = cam_info->K[0];
    focal_y = cam_info->K[4];
    cam_height = cam_info->height;
    cam_width = cam_info->width;
    info = *cam_info;

    ROS_INFO("centre_x = %lf, centre_y = %lf, focal_x = %lf, focal_y = %lf = cam_height = %d, cam_width %d", centre_x,
             centre_y, focal_x, focal_y, cam_height, cam_width);
    published = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/pelican/vi_sensor/camera_depth/camera/image_raw", 1,
  // imageCallback);
  ros::Subscriber sub = nh.subscribe<PointCloud>("/pelican/vi_sensor/camera_depth/depth/points", 1, callback);
  ros::Subscriber cam_sub =
      nh.subscribe<CamInfo>("/pelican/vi_sensor/camera_depth/camera/camera_info", 1, camera_callback);
  publish_result = it.advertise("camera/image", 1);
  ros::spin();
}