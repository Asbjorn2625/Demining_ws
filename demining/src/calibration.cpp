#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

const int max_value_H = 360/2;
const int max_value = 255;
const std::string window_capture_name = "Video Capture";
const std::string window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

cv::Mat HSVImage;
cv::Mat ThreshImage;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string RED_Vision = "detect window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(RED_Vision);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(RED_Vision);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //goes from BGR to HSV filter
    cv::cvtColor(cv_ptr->image , HSVImage , CV_BGR2HSV);
    
    cv::inRange(HSVImage, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V),ThreshImage);


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(window_detection_name, ThreshImage);
    cv::waitKey(3);
  }
};

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = cv::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = cv::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = cv::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = cv::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = cv::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = cv::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Calibration");
    cv::namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    ImageConverter ic;
    ros::spin();
    return 0;
}