#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <imageProcessor.h>
#include <controlConst.h>
using namespace std;
using namespace cv;
double slope(Point first, Point second);

class GetTrainData
{
private:
    ros::NodeHandle n;
    std::string filepath = "/home/redcar/imageTest/frame0";
    Mat img;
    int laneNumber = -1;
    ControlConst cont;

public:
    GetTrainData()
    {
        n = ros::NodeHandle("~");
    }

    void itterateImages()
    {
        for (int i = 177; i < 189; i++)
        {
            std::string thisFile = filepath + std::to_string(i);
            string extension = thisFile + ".jpg";
            ROS_INFO("filename: %s", extension.c_str());
            img = imread(extension, IMREAD_COLOR);
            Mat imageEdit = img.clone();
            ROS_INFO("Read file");

            if(img.empty()){
                ROS_INFO("Cannot Open Image");
            }
            vector<vector<vector<double>>> laneLines = getLaneLines(imageEdit);
            vector<vector<double>> yellowLaneLines = laneLines.at(0);
            vector<vector<double>> whiteLaneLines = laneLines.at(1);
            laneNumber = cont.laneFinder(whiteLaneLines, yellowLaneLines, laneNumber);

            vector<double> speedSteer = cont.steerSpeed(yellowLaneLines, whiteLaneLines, laneNumber);
            double steeringAngle = speedSteer.at(1) * -0.8;

            savePicture(img, thisFile, steeringAngle, i);

        }
    }

    vector<vector<vector<double>>> getLaneLines(Mat img)
    {
        Rect crop(250, 450, 960, 270);
        Mat croppedImage;
        croppedImage = img(crop);

        //White values
        int whmin = 34, whmax = 179, wsmin = 0, wsmax = 255, wvmin = 252, wvmax = 255;

        Scalar wLower(whmin, wsmin, wvmin);
        Scalar wUpper(whmax, wsmax, wvmax);
        //Yellow values
        int yhmin = 19, yhmax = 49, ysmin = 34, ysmax = 126, yvmin = 175, yvmax = 255;

        Scalar yLower(yhmin, ysmin, yvmin);
        Scalar yUpper(yhmax, ysmax, yvmax);

        // namedWindow("Trackbars",(640,200));
        // createTrackbar("Hue min","Trackbars",&hmin,179);
        // createTrackbar("Hue max","Trackbars",&hmax,179);
        // createTrackbar("Sat min","Trackbars",&smin,255);
        // createTrackbar("Sat max","Trackbars",&smax,255);
        // createTrackbar("Val min","Trackbars",&vmin,255);
        // createTrackbar("Val max","Trackbars",&vmax,255);

        ImageProcessor i;
        Mat yBlur = i.getBlur(yLower, yUpper, croppedImage);
        Mat wBlur = i.getBlur(wLower, wUpper, croppedImage);

        Mat yErodeMat = i.getErode(yBlur);
        Mat wErodeMat = i.getErode(wBlur);

        vector<vector<double>> yellowLaneLines;
        vector<vector<double>> whiteLaneLines;

        yellowLaneLines = i.processImage(yErodeMat, yBlur);
        whiteLaneLines = i.processImage(wErodeMat, wBlur);

        vector<vector<vector<double>>> lines;
        lines.push_back(yellowLaneLines);
        lines.push_back(whiteLaneLines);

        return lines;
    }

    void savePicture(Mat img, string filepath, double steeringAngle, int number)
    {
        std::string newFilePath = "/home/redcar/steerImages/frame" + std::to_string(number) + "_SA_" + std::to_string(steeringAngle) + ".jpg";
        bool check = imwrite(newFilePath, img);
        if(check == false){
            ROS_INFO("Could not save image");
        }

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

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "Reactive_Method");
    GetTrainData rw;
    rw.itterateImages();
    ros::spinOnce();
    return 0;
}