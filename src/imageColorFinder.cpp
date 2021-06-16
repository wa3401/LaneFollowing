#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <tf2_msgs/TFMessage.h>
using namespace std;
using namespace cv;
double slope(Point first,Point second);

//used for testing
int hmin = 0,hmax = 179,smin = 0,smax=255,vmin=0,vmax=255;
//  static const std::string OPENCV_WINDOW = "Image window";
//  static const std::string OTHER_WINDOW = "edge window";
 static const std::string TEST_WINDOW = "edge window";
class SensorTOCV{
    private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    ros::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher drive_pub;
    Mat img;
    public:
    SensorTOCV():image_transport(n) {
        n = ros::NodeHandle("~");
        ///camera/depth/image_rect_raw
        ///camera/color/image_raw
         std::string drive_topic;
         n.getParam("/nav_drive_topic", drive_topic);
        image_sub_ = n.subscribe("/tf",1,&SensorTOCV::pose_callback,this);
        image_pub_ = image_transport.advertise("/testOutputCam", 1);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        //  cv::namedWindow(OPENCV_WINDOW);
        //  namedWindow(OTHER_WINDOW);
            namedWindow(TEST_WINDOW);
        //used for testing
         namedWindow("Trackbars",(640,200));
         createTrackbar("Hue min","Trackbars",&hmin,179);
         createTrackbar("Hue max","Trackbars",&hmax,179);
         createTrackbar("Sat min","Trackbars",&smin,255);
         createTrackbar("Sat max","Trackbars",&smax,255);
         createTrackbar("Val min","Trackbars",&vmin,255);
         createTrackbar("Val max","Trackbars",&vmax,255);
        //img = imread("/home/ryan/Downloads/photos/image0.jpeg", IMREAD_COLOR);
        img = imread("/home/redcar/image1.jpg", IMREAD_COLOR);

    }
    ~SensorTOCV()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
    // destroyWindow(OTHER_WINDOW);
    destroyWindow(TEST_WINDOW);
  }
    
  void pose_callback(const tf2_msgs::TFMessage::ConstPtr& poses){
    //subscribes to any random topic that updates frequently
    // I ran a test program that outputted tf data so i used that message type
      cv_bridge::CvImagePtr cv_ptr;
      Mat output;
      //CV_8UC1
    try
    {
      // for color camera
      //for depth camera
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

      // use functions canny() and gaussianblur() to find edges
      //1280 x 720 is the picture size for the color camera;
      //all values
      //int hmin = 0,hmax = 179,smin = 0,smax=255,vmin=0,vmax=255;
      //int hmin = 90,hmax = 115,smin = 75,smax=255,vmin=0,vmax=255;
      Scalar lower(hmin,smin,vmin);
      Scalar upper(hmax,smax,vmax);
      Mat imageInHSV;
      cvtColor(img,imageInHSV,COLOR_BGR2HSV);
      Mat colorFiltered;
      inRange(imageInHSV,lower,upper,colorFiltered);
      output =colorFiltered;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //
    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, output);
    // imshow(OTHER_WINDOW,displayForLines);
    cv::imshow(TEST_WINDOW, output);
    //wait time is in miliseconds
    cv::waitKey(3);
    //test stuff
  }
};

double slope(Point first,Point second){
  // slope is taken such that horizontal side of camera is y axis where right side is positive and left side is negative
  // vertical side of camera is x axis where top side would be positve and bottom side is negative
  // so 0,0 to 1,1 should output -1
  // any line leaning left in the camera will have a negative slope
  // any line leaning right in the camera should have a positive slope
  // slopes close to zero are straight lines
  double secondX = second.x;
  double secondY = second.y;
  double firstX  = first.x;
  double firstY = first.y;
    if(second.y - first.y == 0 ){
      return 0;
    }
    return (-1.0 * (second.x - first.x )) / (second.y - first.y);
}
int main(int argc, char ** argv) {
    ros::init(argc, argv, "Reactive_Method");
    SensorTOCV rw;
    ros::spin();
    return 0;
}