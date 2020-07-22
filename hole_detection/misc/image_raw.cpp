#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  /*float* depths = (float*)(&msg->data[0]);

  // Image coordinates of the center pixel
  int u = msg->width / 2;
  int v = msg->height / 2;
*/
  // Linear index of the center pixel
  // int centerIdx = u + msg->width * v;
  float middlepoint = (float)msg->data[msg->width * msg->height / 2] * 0.001f;

  // Output the measure
  // ROS_INFO("Center distance : %g m", depths[centerIdx]);
  ROS_INFO("distance %f", middlepoint);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/pelican/vi_sensor/camera_depth/camera/image_raw", 1, imageCallback);
  ros::spin();
}