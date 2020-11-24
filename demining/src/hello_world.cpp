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

static const std::string OPENCV_WINDOW = "Image window";
static const std::string Blurred_Vision = "Blur";
static const std::string edged_Vision = "detect edges";

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
    cv::namedWindow(Blurred_Vision);
    cv::namedWindow(edged_Vision);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(Blurred_Vision);
    cv::destroyWindow(edged_Vision);
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
    //Scans the image for red, the Scalar is made with the calibration.cpp
     cv::inRange(HSVImage, cv::Scalar(0, 177, 92), cv::Scalar(180, 255, 255), ThreshImage);
     //blurs the image to filter the noise
     cv::blur(ThreshImage, Blurimage, cv::Size(3,3) );
     //Makes the lines more visible
    cv::Canny(Blurimage, Edgedimage, 50, 200, 3);
    
    //We will make an vector named contour which we can use to analyse our matrix
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Edgedimage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    if(contours.size() != 0){
      std::vector<std::vector<cv::Point>> contours_poly(contours.size());
      std::vector <cv::Rect> boundRect(contours.size());
      std::vector <cv::Point2f> centers(contours.size());
      std::vector <float> radius(contours.size());

      for( std::size_t i=0 ; i < contours.size() ; i++ ){
        cv::approxPolyDP(contours[i], contours_poly[i],3,true);
        boundRect[i] = cv::boundingRect(contours_poly[i]);
      };
// draws an drawing on the display
cv::Mat drawing = cv::Mat::zeros(Edgedimage.size() , CV_8UC3);
cv::Scalar marxred(237,27,36);
      for( std::size_t i=0 ; i < contours.size() ; i++ ){
        cv::drawContours(drawing,contours_poly,int(i),marxred);
        cv::rectangle(drawing,boundRect[i].tl(),boundRect[i].br(),marxred);
      };
          cv::imshow(Blurred_Vision , drawing );
    };



    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(edged_Vision , Edgedimage);
    cv::waitKey(3);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
      auto count = cv::countNonZero(Blurimage);
    std::cout << "Nonzeros:" << count << std::endl;
  ros::spin();
  return 0;
}