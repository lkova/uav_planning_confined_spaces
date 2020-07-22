#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

image_transport::CameraSubscriber cam_sub;
image_geometry::PinholeCameraModel cam_model;
bool cam_info_found = false;
image_transport::Publisher cam_pub;
ros::Publisher pub_point;

void cam_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg)
{
    if (!cam_info_found)
    {
        if (info_msg->K[0] > 1.0)
        {
            cam_info_found = true;
            cam_model.fromCameraInfo(info_msg);
        }
    }
    else
    {

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat imgGrayscale = cv_ptr->image;
        //cv::Mat imgGrayscale; // grayscale of input image
        cv::Mat imgBlurred; // intermediate blured image
        cv::Mat imgCanny;   // Canny edge image

        //cv::cvtColor(image, imgGrayscale, CV_BGR2GRAY);
        int morph_size = 4;

        cv::Mat element = getStructuringElement(cv::MORPH_CROSS, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                                cv::Point(morph_size, morph_size));

        dilate(imgGrayscale, imgGrayscale, element);
        erode(imgGrayscale, imgGrayscale, element);

        /*cv::GaussianBlur(imgGrayscale,   // input image
                     imgBlurred,     // output image
                     cv::Size(5, 5), // smoothing window width and height in pixels
                     0.0);           // sigma value, determines how much the image will be blurred*/

        cv::Canny(imgGrayscale, // input image
                  imgCanny,     // output image
                  30,           // low threshold
                  200);         // high threshold

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        findContours(imgCanny, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // std::cout << "hier " << contours.size() << std::endl;

        int largest_contour_index = 0;
        int i = 0;
        double area = 0.0;

        std::vector<cv::Point> contour2;
        for (std::vector<cv::Point> cnt : contours)
        {

            if (contourArea(cnt) > area)
            {
                area = contourArea(cnt);
                largest_contour_index = i;
            }
            i = i + 1;
        }

        /*cv::Moments moments;
        double M00, M01, M10;
        moments = cv::moments(contours[largest_contour_index]);
        M00 = moments.m00;
        M10 = moments.m10;
        M01 = moments.m01;
        double cx = (int)(M10 / M00);
        double cy = (int)(M01 / M00);
        cv::Point2d uv(cx, cy);
        cv::Point3d pt_cv;
        pt_cv = cam_model.projectPixelTo3dRay(uv);
        cv::circle(imgGrayscale, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);*/

        double cx = 0.0;
        double cy = 0.0;
        for (cv::Point p : contours[largest_contour_index])
        {

            /*cv::Point2d uvc(p.x, p.y);
            cv::Point3d pt;
            pt = cam_model.projectPixelTo3dRay(uvc);

            double dd = sqrt(pow(pt_cv.x - pt.x, 2) + pow(pt_cv.y - pt.y, 2));
            if (abs(dd) > dist_max)
            {
                dist_max = abs(dd);
            }*/

            if (p.y > cy)
            {
                cx = p.y;
                cy = p.x;
            }
        }
        cv::Point2d uv(cx, cy);
        cv::Point3d pt_cv;
        pt_cv = cam_model.projectPixelTo3dRay(uv);
        cv::circle(imgGrayscale, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_8);

        //ROS_INFO("x %lf y %lf z %lf", pt_cv.x, pt_cv.y, pt_cv.z);
        //ROS_INFO("x %lf y %lf ", cx, cy);
        // double per = cv::arcLength(cnt2, true);
        /* cv::Point2d uv(212, 151);
    cv::Point3d pt_cv;
    pt_cv = cam_model.projectPixelTo3dRay(uv);*/

        //double area = contourArea(contours[largest_contour_index]);

        //cv::drawContours(imgGrayscale, contours, 0, cvScalar(0, 255, 0), 2);
        //cv::namedWindow("imgOriginal", CV_WINDOW_AUTOSIZE);
        //cv::namedWindow("imgCanny", CV_WINDOW_AUTOSIZE);

        // Show windows
        //cv::imshow("imgOriginal", imgGrayscale);
        //cv::imshow("imgCanny", imgCanny);

        //cv::waitKey(3); // hold windows open until user presses a key
        //ROS_INFO("juhu %lf", per);
        //std::cout << per << std::endl;
        //cam_pub.publish(cv_ptr->toImageMsg());
        geometry_msgs::PointStamped po;
        po.point.x = pt_cv.x;
        po.point.y = pt_cv.y;
        po.point.z = pt_cv.z;
        po.header = image_msg->header;
        pub_point.publish(po);
        /*for (int i = 0; cv_ptr->image.rows; i++)
        {
            for (int j = 0; cv_ptr->image.cols; j++)
            {
                float dist_val = cv_ptr->image.at<float>(cx, cy);
                if (dist_val > 0.0)
                    ROS_INFO("dist_val %lf", dist_val);
            }
        }*/
        float dist_val = cv_ptr->image.at<float>(cx, cy);
        float dist_val2 = cv_ptr->image.at<float>(cx + 200, cy + 200);
        ROS_INFO("dist_val %lf, kurac %lf", dist_val, dist_val2);
        //cam_pub.publish(cv_ptr->toImageMsg());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    cam_sub = it.subscribeCamera("/pelican/vi_sensor/camera_depth/camera/image_raw", 1, cam_callback);
    cam_pub = it.advertise("/pelican/vi_sensor/camera_depth/camera/image_raw", 1);
    pub_point = nh.advertise<geometry_msgs::PointStamped>("point_dept", 1);
    //cam_sub = it.subscribeCamera("/pelican/vi_sensor/left/image_raw", 1, cam_callback);
    ros::spin();

    return 0;
}
