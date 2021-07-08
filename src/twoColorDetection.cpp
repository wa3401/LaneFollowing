/**
 * @Author Ryan Brown, Will Anderson
 * 
 * This is a modification of the rosImageToCV.cpp file in this folder. 
 * This file processes the white and yellow lines into different images so that the car can tell which lane it is in, improving the lane following algorithm.
 * 
 * ^^^^^USE DEBUGER TO RUN THIS FILE^^^^^
 **/

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

//used for testing
//int hmin = 0,hmax = 179,smin = 0,smax=255,vmin=0,vmax=255;
static const std::string OPENCV_WINDOW_YELLOW = "Image yellow window";
static const std::string OPENCV_WINDOW_WHITE = "Image white window";
static const std::string OTHER_WINDOW_YELLOW = "edge window yellow";
static const std::string OTHER_WINDOW_WHITE = "edge window white";
class SensorTOCV
{
private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher drive_pub;
    //Right lane will be 0, Left lane 1
    int laneNumber = -1;
    double prevWhiteX = 465;
    double prevYellowX = 465;
    double steeringAngle = 0;
    double carSpeed = 0;

public:
    SensorTOCV() : image_transport(n)
    {
        n = ros::NodeHandle("~");
        ///camera/depth/image_rect_raw
        ///camera/color/image_raw
        std::string drive_topic;
        drive_topic = "/vesc/low_level/ackermann_cmd_mux/input/navigation";

        // Subscribes to this topic, which is published by realsense camera node (This is started in the launch file)
        image_sub_ = image_transport.subscribe("/camera/color/image_raw", 1, &SensorTOCV::image_callback, this);
        image_pub_ = image_transport.advertise("/testOutputCam", 1);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        cv::namedWindow(OPENCV_WINDOW_YELLOW);
        namedWindow(OPENCV_WINDOW_WHITE);
        namedWindow(OTHER_WINDOW_YELLOW);
        namedWindow(OTHER_WINDOW_WHITE);
    }
    ~SensorTOCV()
    {
        cv::destroyAllWindows();
    }

    // Called every time an image is published to this topic
    void image_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        //Creates an open cv bridge
        cv_bridge::CvImagePtr cv_ptr;
        //Create opencv mats for the color detection images after canny edge detection
        Mat woutput;
        Mat youtput;
        //Creates the displays for the hough lines
        Mat displayForYellowLines(270, 960, CV_8UC3, Scalar(0, 0, 0));
        Mat displayForWhiteLines(270, 960, CV_8UC3, Scalar(0, 0, 0));

        //CV_8UC1
        try
        {

            /** -------------------------------------------------**\
            * ------------------Image Processing------------------ *
            \**--------------------------------------------------**/

            // for color camera
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            //for depth camera
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

            // use functions canny() and gaussianblur() to find edges
            //1280 x 720 is the picture size for the color camera
            Rect crop(250, 450, 960, 270);
            Mat croppedImage;
            croppedImage = cv_ptr->image(crop);

            //White mins and maxes for Hue-Saturation-Value model
            int whmin = 34, whmax = 179, wsmin = 0, wsmax = 255, wvmin = 252, wvmax = 255;

            Scalar wLower(whmin, wsmin, wvmin);
            Scalar wUpper(whmax, wsmax, wvmax);
            //Yellow mins and maxes for Hue-Saturation-Value model
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

            //Create ImageProcessor (From Header file)
            ImageProcessor i;

            //Gets the blurred images for they white and yellow lines
            Mat yBlur = i.getBlur(yLower, yUpper, croppedImage);
            Mat wBlur = i.getBlur(wLower, wUpper, croppedImage);

            //Gets the erroded images from canny edge detection (Images shown in black and white when file is run)
            Mat yErodeMat = i.getErode(yBlur);
            Mat wErodeMat = i.getErode(wBlur);

            /** -------------------------------------------------**\
            * -------------------Line Detection------------------- *
            \**--------------------------------------------------**/

            //Create vectors to hold slopes and intercepts of lines found with Hough Transform
            vector<vector<double> > yellowLaneLines;
            vector<vector<double> > whiteLaneLines;

            //Processes images using Hough Transform and adds all slopes and intercepts along the bottom of image to the vector
            yellowLaneLines = i.processImage(yErodeMat, yBlur);
            whiteLaneLines = i.processImage(wErodeMat, wBlur);

            //PRINTS ALL INTERCEPTS AND SLOPES OF YELLOW LINES
            for (int i = 0; i < static_cast<int>(yellowLaneLines.size()); i++)
            {
                ROS_INFO("Yellow X coordinate: %s", std::to_string(yellowLaneLines.at(i).at(0)).c_str());
                ROS_INFO("Yellow Slope: %s", std::to_string(yellowLaneLines.at(i).at(1)).c_str());
            }

            //PRINTS ALL INTERCEPTS AND SLOPES OF WHITE LINES
            for (int i = 0; i < static_cast<int>(whiteLaneLines.size()); i++)
            {
                ROS_INFO("White X coordinate: %s", std::to_string(whiteLaneLines.at(i).at(0)).c_str());
                ROS_INFO("White Slope: %s", std::to_string(whiteLaneLines.at(i).at(1)).c_str());
            }

            //Finds total number of lanes found
            int numLanes = static_cast<int>(whiteLaneLines.size()) + static_cast<int>(yellowLaneLines.size());
            ROS_INFO("NUMBER OF LANES FOUND: %s", std::to_string(numLanes).c_str());


            /** -------------------------------------------------**\
            * ----------------Steering Calculation----------------- *
            \**--------------------------------------------------**/

            
            //IF BOTH COLORS OF LANES ARE FOUND, REDETERMINE LANE (USING FUNCTION DEFINED BELOW)
            if (static_cast<int>(whiteLaneLines.size()) >= 1 && static_cast<int>(yellowLaneLines.size()) >= 1)
            {
                laneNumber = laneFinder(whiteLaneLines, yellowLaneLines, laneNumber);
            }
            ROS_INFO("Lane Number: %s", std::to_string(laneNumber).c_str());

            //CREATE STEERING CONTROLER FOR CONSTANT SPEED
            ControlConst c;

            //Function returns steering angle and speed (always 0.75 in this case) based on line lines and lane number
            vector<double> speedSteer = c.steerSpeed(yellowLaneLines, whiteLaneLines, laneNumber);
            carSpeed = speedSteer.at(0);
            steeringAngle = speedSteer.at(1);

            //Sets output images
            woutput = wErodeMat;
            youtput = yErodeMat;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        ROS_INFO("Angle: %s", std::to_string(steeringAngle).c_str());
        ROS_INFO("Car Speed: %s", std::to_string(carSpeed).c_str());


        /** -------------------------------------------------**\
        * ---------------------Publishing--------------------- *
        \**--------------------------------------------------**/

        //UPLOAD DRIVE DATA
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = steeringAngle * -0.8;
        drive_msg.speed = carSpeed;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW_WHITE, woutput);
        cv::imshow(OPENCV_WINDOW_YELLOW, youtput);
        imshow(OTHER_WINDOW_WHITE, displayForWhiteLines);
        imshow(OTHER_WINDOW_YELLOW, displayForYellowLines);

        //wait time is in miliseconds
        cv::waitKey(3);

        //test stuff
        cv_bridge::CvImage yout_msg;
        yout_msg.header = msg->header;                                // Same timestamp and tf frame as input image
        yout_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        yout_msg.image = youtput;                                     // Your cv::Mat

        // Output modified video stream
        image_pub_.publish(yout_msg.toImageMsg());

        cv_bridge::CvImage wout_msg;
        wout_msg.header = msg->header;                                // Same timestamp and tf frame as input image
        wout_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
        wout_msg.image = woutput;                                     // Your cv::Mat

        // Output modified video stream
        image_pub_.publish(wout_msg.toImageMsg());
    }

    //FINDS WHICH LANE THE CAR IS CURRENTLY IN
    int
    laneFinder(vector<vector<double> > wLines, vector<vector<double> > yLines, int lane)
    {
        //USES THE X COORDINATE OF EACH OF THE LINES TO DETERMINE LANE
        int laneFind = lane;
        if (yLines.size() > 0)
        {
            if (yLines.at(0).at(0) > 480)
            {
                laneFind = 0;
            }
            else
            {
                laneFind = 1;
            }
        }
        else if (wLines.size() > 0)
        {
            if (wLines.at(0).at(0) < 480)
            {
                laneFind = 0;
            }
            else
            {
                laneFind = 1;
            }
        }
        return laneFind;
    }
};

double slope(Point first, Point second)
{
    // slope is taken such that horizontal side of camera is y axis where right side is positive and left side is negative
    // vertical side of camera is x axis where top side would be positve and bottom side is negative
    // so 0,0 to 1,1 should output -1
    // any line leaning left in the camera will have a negative slope
    // any line leaning right in the camera should have a positive slope
    // slopes close to zero are straight yellowLines
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
