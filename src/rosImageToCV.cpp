#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
using namespace std;
using namespace cv;
double slope(Point first, Point second);

//used for testing
//int hmin = 0,hmax = 179,smin = 0,smax=255,vmin=0,vmax=255;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OTHER_WINDOW = "edge window";
static const std::string BAR_WINDOW = "bar window";
class SensorTOCV
{
private:
  ros::NodeHandle n;
  image_transport::ImageTransport image_transport;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher drive_pub;

public:
  SensorTOCV() : image_transport(n)
  {
    n = ros::NodeHandle("~");
    ///camera/depth/image_rect_raw
    ///camera/color/image_raw
    std::string drive_topic;
    drive_topic = "/vesc/low_level/ackermann_cmd_mux/input/navigation";
    image_sub_ = image_transport.subscribe("/camera/color/image_raw", 1, &SensorTOCV::image_callback, this);
    image_pub_ = image_transport.advertise("/testOutputCam", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    cv::namedWindow(OPENCV_WINDOW);
    namedWindow(OTHER_WINDOW);
    namedWindow(BAR_WINDOW);

    //used for testing
    // namedWindow("Trackbars",(640,200));
    // createTrackbar("Hue min","Trackbars",&hmin,179);
    // createTrackbar("Hue max","Trackbars",&hmax,179);
    // createTrackbar("Sat min","Trackbars",&smin,255);
    // createTrackbar("Sat max","Trackbars",&smax,255);
    // createTrackbar("Val min","Trackbars",&vmin,255);
    // createTrackbar("Val max","Trackbars",&vmax,255);
  }
  ~SensorTOCV()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    destroyWindow(OTHER_WINDOW);
    destroyWindow(BAR_WINDOW);
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    Mat output;
    Mat displayForLines(270, 930, CV_8UC3, Scalar(0, 0, 0));
    double steeringAngle = 0;
    double carSpeed = 0;
    //CV_8UC1
    try
    {
      // for color camera
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //for depth camera
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

      // use functions canny() and gaussianblur() to find edges
      //1280 x 720 is the picture size for the color camera
      Rect crop(250, 450, 930, 270);
      Mat croppedImage;
      croppedImage = cv_ptr->image(crop);

      Mat yuv_img;
      cvtColor(croppedImage, yuv_img, CV_BGR2YUV);
      std::vector<Mat> channels;
      split(yuv_img, channels);
      equalizeHist(channels[0], channels[0]);
      merge(channels, yuv_img);
      cvtColor(yuv_img, croppedImage, CV_YUV2BGR);
      //white values
      int hmin, hmax, smin, smax, vmin, vmax;
      //int whmin = 0, whmax = 133, wsmin = 0, wsmax = 19, wvmin = 255, wvmax = 255;
      //Yellow values
      //int yhmin = 17, yhmax = 41, ysmin = 72, ysmax = 123, yvmin = 118, yvmax = 255;
      namedWindow("Trackbars", (640, 200));
      createTrackbar("Hue min", "Trackbars", &hmin, 179);
      createTrackbar("Hue max", "Trackbars", &hmax, 179);
      createTrackbar("Sat min", "Trackbars", &smin, 255);
      createTrackbar("Sat max", "Trackbars", &smax, 255);
      createTrackbar("Val min", "Trackbars", &vmin, 255);
      createTrackbar("Val max", "Trackbars", &vmax, 255);

      Scalar lower(hmin, smin, vmin);
      Scalar upper(hmax, smax, vmax);
      Mat imageInHSV;
      cvtColor(croppedImage, imageInHSV, COLOR_BGR2HSV);
      Mat colorFiltered;
      inRange(imageInHSV, lower, upper, colorFiltered);
      Mat blurredImage;
      GaussianBlur(colorFiltered, blurredImage, Size(5, 5), 0);
      Mat edgeImage;
      Canny(blurredImage, edgeImage, 100, 200);
      Mat dilatedImage;
      Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
      Mat extraBlurMat;
      dilate(edgeImage, dilatedImage, kernel);
      GaussianBlur(dilatedImage, extraBlurMat, Size(7, 7), 0);
      Mat erodeMat;
      erode(extraBlurMat, erodeMat, kernel);
      if (blurredImage.size().height > 20)
      {
        vector<Vec4i> lines;
        vector<vector<Point>> pointsForLines;
        HoughLinesP(erodeMat, lines, 1, CV_PI / 180, 70, 30, 10);

        for (size_t i = 0; i < lines.size(); i++)
        {
          //line(displayForLines, Point(lines[i][0], lines[i][1]),Point( lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
          if (lines[i][1] > 240 || lines[i][3] > 240)
          {
            line(displayForLines, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
            vector<Point> newPoint;
            Point beginning(lines[i][0], lines[i][1]);
            Point end(lines[i][2], lines[i][3]);
            if (end.y > beginning.y)
            {
              newPoint.push_back(end);
              newPoint.push_back(beginning);
            }
            else
            {
              newPoint.push_back(beginning);
              newPoint.push_back(end);
            }
            pointsForLines.push_back(newPoint);
          }
        }
        vector<int> countOfLinesAddedToEachLane;
        vector<vector<double>> laneLines;
        for (int j = 0; j < static_cast<int>(pointsForLines.size()); j++)
        {
          int indexOfCorrespondence = -1;
          for (int i = 0; i < static_cast<int>(laneLines.size()); i++)
          {
            //averages the slope and x distance of all lines in the same area
            if (abs(pointsForLines.at(j).at(0).x - laneLines.at(i).at(0)) < 300 && static_cast<int>(laneLines.size()) < 10)
            {
              //test
              double oldX = laneLines.at(i).at(0);
              double oldSlope = laneLines.at(i).at(1);
              double newSlope = slope(pointsForLines.at(j).at(0), pointsForLines.at(j).at(1));
              double newX = pointsForLines.at(j).at(0).x;
              laneLines.at(i).at(0) = (laneLines.at(i).at(0) * countOfLinesAddedToEachLane.at(i) + pointsForLines.at(j).at(0).x) / (countOfLinesAddedToEachLane.at(i) + 1);
              laneLines.at(i).at(1) = (laneLines.at(i).at(1) * countOfLinesAddedToEachLane.at(i) + slope(pointsForLines.at(j).at(0), pointsForLines.at(j).at(1))) / (countOfLinesAddedToEachLane.at(i) + 1);

              double averageX = laneLines.at(i).at(0);
              double averageSlope = laneLines.at(i).at(1);

              countOfLinesAddedToEachLane.at(i)++;
              indexOfCorrespondence = j;
            };
          }
          if (indexOfCorrespondence == -1)
          {
            // add first line when it doesnt match any of the others that already exist
            vector<double> xPointAndSlope;
            xPointAndSlope.push_back(pointsForLines.at(j).at(0).x);
            xPointAndSlope.push_back(slope(pointsForLines.at(j).at(0), pointsForLines.at(j).at(1)));
            laneLines.push_back(xPointAndSlope);
            countOfLinesAddedToEachLane.push_back(1);
          }
        }
        //output of lane lines
        ROS_INFO("Number lanes found: %s", std::to_string(laneLines.size()).c_str());
        for (int i = 0; i < static_cast<int>(laneLines.size()); i++)
        {
          ROS_INFO("X coordinate: %s", std::to_string(laneLines.at(i).at(0)).c_str());
          ROS_INFO("Slope: %s", std::to_string(laneLines.at(i).at(1)).c_str());
        }
        //Calculation of steering angle from lanes found
        //determines how quickly it will restore the car to its mid point in the lane
        // implement this at a later date so it will try to stay in the center of the lane instead of just in between the lines
        double laneMidPointCorrectionCoefficient = (M_PI / 6) / 640;
        double visualAngleCoefficient = 1.0 / 9;
        //2 lanes found
        if (static_cast<int>(laneLines.size()) == 2)
        {
          double YClose = 0;
          double yDistance = 200; //arbitrary decision that works well
          double midPointXClose = (laneLines.at(0).at(0) + laneLines.at(1).at(0)) / 2;
          double lane1XFar = laneLines.at(0).at(0) + laneLines.at(0).at(1) * yDistance;
          double lane2XFar = laneLines.at(1).at(0) + laneLines.at(1).at(1) * yDistance;
          double midPointXFar = (lane1XFar + lane2XFar) / 2;
          line(displayForLines, Point(midPointXClose, 270 - YClose), Point(midPointXFar, 270 - yDistance), Scalar(0, 255, 0), 3, 8);

          double xDistance = abs(midPointXFar - midPointXClose);
          double correctionAngle;
          if (xDistance > 5)
          {
            if (midPointXFar > midPointXClose)
            {
              correctionAngle = M_PI_2 - atan(yDistance / xDistance);
            }
            else
            {
              correctionAngle = -1 * (M_PI_2 - atan(yDistance / xDistance));
            }
          }
          else
          {
            correctionAngle = 0;
          }
          //change later to adjust based on closeness to center of lane
          //change later to adjust based on closeness to center of lane
          double distanceFromCenter = abs(640 - midPointXClose);
          double distanceFromCenterAngleCorrection;
          if (640 - midPointXClose > 0)
          {
            distanceFromCenterAngleCorrection = -1 * distanceFromCenter * laneMidPointCorrectionCoefficient;
          }
          else
          {
            distanceFromCenterAngleCorrection = distanceFromCenter * laneMidPointCorrectionCoefficient;
          }
          steeringAngle = correctionAngle * visualAngleCoefficient + distanceFromCenterAngleCorrection;
          carSpeed = 0.5;
        }

        // else if(static_cast<int>(laneLines.size()) > 2){
        //   int leftLineIdx = -1;
        //   int rightLineIdx = -1;
        //   for(int i = 0; i < static_cast<int>(laneLines.size()); i++){
        //     if(rightLineIdx < 0 && laneLines.at(i).at(1) < 0){
        //       leftLineIdx = i - 1;
        //       rightLineIdx = i;
        //     }
        //   }
        //   double YClose = 0;
        //   double yDistance = 200; //arbitrary decision that works well
        //   double midPointXClose = (laneLines.at(leftLineIdx).at(0) + laneLines.at(rightLineIdx).at(0))/2;
        //   double lane1XFar = laneLines.at(leftLineIdx).at(0) + laneLines.at(leftLineIdx).at(1) * yDistance;
        //   double lane2XFar = laneLines.at(rightLineIdx).at(0) + laneLines.at(rightLineIdx).at(1) * yDistance;
        //   double midPointXFar = (lane1XFar + lane2XFar) / 2;
        //   line(displayForLines, Point(midPointXClose, 270 -YClose),Point(midPointXFar,270- yDistance), Scalar(0,255,0), 3, 8 );

        //   double xDistance = abs(midPointXFar - midPointXClose);
        //   double correctionAngle;
        //   if(xDistance >5){
        //     if(midPointXFar>midPointXClose){
        //       correctionAngle = M_PI_2 - atan(yDistance/xDistance);
        //     }
        //     else{
        //        correctionAngle = -1 * (M_PI_2 - atan(yDistance/xDistance));
        //     }
        //   }else{
        //     correctionAngle = 0;
        //   }
        //   //change later to adjust based on closeness to center of lane
        //   //change later to adjust based on closeness to center of lane
        //   double distanceFromCenter = abs(640 - midPointXClose);
        //   double distanceFromCenterAngleCorrection;
        //   if(640 - midPointXClose >0){
        //     distanceFromCenterAngleCorrection = -1*distanceFromCenter*laneMidPointCorrectionCoefficient;
        //   }else{
        //     distanceFromCenterAngleCorrection = distanceFromCenter*laneMidPointCorrectionCoefficient;
        //   }
        //   steeringAngle = correctionAngle* visualAngleCoefficient + distanceFromCenterAngleCorrection;
        //   carSpeed = 0.5;

        // }

        // 1 Lane found
        else if (static_cast<int>(laneLines.size()) == 1)
        {
          double yClose = 0;
          double xClose = laneLines.at(0).at(0);
          double yFar = 200;
          double xFar = laneLines.at(0).at(0) + yFar * laneLines.at(0).at(1);
          double correctionAngle;
          double xDistance = abs(xFar - xClose);
          if (xDistance > 5)
          {
            if (xFar > xClose)
            {
              correctionAngle = M_PI_2 - atan(yFar / xDistance);
            }
            else
            {
              correctionAngle = -1 * (M_PI_2 - atan(yFar / xDistance));
            }
          }
          else
          {
            correctionAngle = 0;
          }
          //change later to adjust based on closeness to center of lane
          double distanceFromCenterAngleCorrection = 0;
          if (laneLines.at(0).at(1) < 0)
          {
            distanceFromCenterAngleCorrection = -1 * max(0.0, (1280 - xClose - 250)) * laneMidPointCorrectionCoefficient;
          }
          else
          {
            distanceFromCenterAngleCorrection = max(0.0, (xClose - 250)) * laneMidPointCorrectionCoefficient;
          }
          //leaning left line has negative slope
          //line goes left and x > mid point means the car should turn left
          //line goes right
          steeringAngle = correctionAngle * visualAngleCoefficient + distanceFromCenterAngleCorrection;
          carSpeed = 0.5;
        }
      }
      output = erodeMat;
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    ROS_INFO("Angle: %s", std::to_string(steeringAngle).c_str());
    ROS_INFO("Car Speed: %s", std::to_string(carSpeed).c_str());
    //UPLOAD DRIVE DATA
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.steering_angle = steeringAngle * -0.8;
    drive_msg.speed = 0.75;
    drive_st_msg.drive = drive_msg;
    drive_pub.publish(drive_st_msg);
    //
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, output);
    imshow(OTHER_WINDOW, displayForLines);
    //wait time is in miliseconds
    cv::waitKey(3);
    //test stuff
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;                                // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image = output;                                      // Your cv::Mat

    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
  }
};

double slope(Point first, Point second)
{
  // slope is taken such that horizontal side of camera is y axis where right side is positive and left side is negative
  // vertical side of camera is x axis where top side would be positve and bottom side is negative
  // so 0,0 to 1,1 should output -1
  // any line leaning left in the camera will have a negative slope
  // any line leaning right in the camera should have a positive slope
  // slopes close to zero are straight lines
  double secondX = second.x;
  double secondY = second.y;
  double firstX = first.x;
  double firstY = first.y;
  if (second.y - first.y == 0)
  {
    return 0;
  }
  return (-1.0 * (second.x - first.x)) / (second.y - first.y);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Reactive_Method");
  SensorTOCV rw;
  ros::spin();
  return 0;
}