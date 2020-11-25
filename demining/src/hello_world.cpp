#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

cv::Mat HSVImage;
cv::Mat ThreshImage;
cv::Mat Blurimage;
cv::Mat Edgedimage;
cv::Point P1;
cv::Point P2;
cv::Point P3;
cv::Point P4;
cv::Point P5;
cv::Point P6;
cv::Point P7;
cv::Point P8;
cv::Scalar red(4,0,255);
cv::Scalar NavyBlue(237, 27, 36);

static const std::string OPENCV_WINDOW = "Image window";
static const std::string Hamburg = "HSV Image";


class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  //Opencv need an special format for their pictures so in this function we are making a bridge so that the openCV library can read the ros sensor data.
  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //goes from BGR to HSV filter
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);


   
    //draws our detection window
    P1.x = 140;
    P1.y = 432;
    P2.x = 502;
    P2.y = 204;
    P3.x = 0;
    P3.y = 479;
    P4.x = P1.x;
    P4.y = 0;
    P5.x = 639;
    P5.y = P1.y;
    P6.x = P2.x;
    P6.y = P3.y;
    P7.x = P5.x;
    P7.y = P4.y;
    P8.x = P3.x;
    P8.y= P2.y;

    //kanter rundt om vores detection window
     cv::rectangle(HSVImage,P3, P4 , red,-1);
     cv::rectangle(HSVImage,P3, P5 , red,-1);
     cv::rectangle(HSVImage,P6, P7 , red,-1);
     cv::rectangle(HSVImage,P7, P8 , red,-1);
     

    cv::rectangle(cv_ptr->image,P1,P2, red);

    //Scans the image for red, the Scalar is calibrated with the calibration.cpp
    cv::inRange(HSVImage, cv::Scalar(47, 79, 111), cv::Scalar(104, 255, 255), ThreshImage);
    //blurs the image to filter the noise
    cv::blur(ThreshImage, Blurimage, cv::Size(3, 3));
    //Makes the lines more visible
    cv::Canny(Blurimage, Edgedimage, 50, 200, 3);

    //We will make an vector named contour which we can use to analyse our matrix
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Edgedimage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if (contours.size() != 0)
    {
      std::vector<std::vector<cv::Point>> contours_poly(contours.size());
      std::vector<cv::Rect> boundRect(contours.size());


      for (std::size_t i = 0; i < contours.size(); i++)
      {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
       double area = cv::contourArea(contours[i]);
        


        // Drawing the contours on our image window
        cv::drawContours(cv_ptr->image, contours_poly, int(i), NavyBlue);
        cv::rectangle(cv_ptr->image, boundRect[i].tl(), boundRect[i].br(), NavyBlue);
      };
    };

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  };
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}