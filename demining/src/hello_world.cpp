#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


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
    //For red i think
     cv::inRange(HSVImage, cv::Scalar(0, 177, 92), cv::Scalar(180, 255, 255), ThreshImage);


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(RED_Vision, HSVImage);
    cv::waitKey(3);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}