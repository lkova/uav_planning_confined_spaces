#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef sensor_msgs::CameraInfo CamInfo;

image_transport::CameraSubscriber cam_sub;

void cam_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try
    {
        input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        image = input_bridge->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR("[draw_frames] Failed to convert image");
        return;
    }
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);
    cv::Point2d uv(212, 151);
    cv::Point3d pt_cv;
    pt_cv = cam_model.projectPixelTo3dRay(uv);

    ROS_INFO("uv %lf %lf, x %lf, y %lf, z %lf, check par %lf", uv.x, uv.y, pt_cv.x, pt_cv.y, pt_cv.z, info_msg->K[0]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    cam_sub = it.subscribeCamera("/pelican/vi_sensor/camera_depth/camera/image_raw", 1, cam_callback);
    ros::spin();
}