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
        image_sub_ = image_transport.subscribe("/camera/color/image_raw", 1, &SensorTOCV::image_callback, this);
        image_pub_ = image_transport.advertise("/testOutputCam", 1);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        cv::namedWindow(OPENCV_WINDOW_YELLOW);
        namedWindow(OPENCV_WINDOW_WHITE);
        namedWindow(OTHER_WINDOW_YELLOW);
        namedWindow(OTHER_WINDOW_WHITE);

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
        cv::destroyAllWindows();
    }

    void image_callback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        Mat woutput;
        Mat youtput;
        Mat displayForYellowLines(270, 960, CV_8UC3, Scalar(0, 0, 0));
        Mat displayForWhiteLines(270, 960, CV_8UC3, Scalar(0, 0, 0));
        
        //CV_8UC1
        try
        {
            // for color camera
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            //for depth camera
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

            // use functions canny() and gaussianblur() to find edges
            //1280 x 720 is the picture size for the color camera
            Rect crop(250, 450, 960, 270);
            Mat croppedImage;
            croppedImage = cv_ptr->image(crop);

            Mat yuv_img;
            cvtColor(croppedImage, yuv_img, CV_BGR2YUV);
            std::vector<Mat> channels;
            split(yuv_img, channels);
            equalizeHist(channels[0], channels[0]);
            merge(channels, yuv_img);
            cvtColor(yuv_img, croppedImage, CV_YUV2BGR);

            // namedWindow("Trackbars",(640,200));
            // createTrackbar("Hue min","Trackbars",&hmin,179);
            // createTrackbar("Hue max","Trackbars",&hmax,179);
            // createTrackbar("Sat min","Trackbars",&smin,255);
            // createTrackbar("Sat max","Trackbars",&smax,255);
            // createTrackbar("Val min","Trackbars",&vmin,255);
            // createTrackbar("Val max","Trackbars",&vmax,255);

            //White values
            int whmin = 34, whmax = 179, wsmin = 0, wsmax = 255, wvmin = 252, wvmax = 255;

            Scalar wLower(whmin, wsmin, wvmin);
            Scalar wUpper(whmax, wsmax, wvmax);
            //Yellow values
            int yhmin = 19, yhmax = 49, ysmin = 34, ysmax = 126, yvmin = 175, yvmax = 255;

            Scalar yLower(yhmin, ysmin, yvmin);
            Scalar yUpper(yhmax, ysmax, yvmax);

            //Yellow Line Detection
            Mat imageInHSV;
            cvtColor(croppedImage, imageInHSV, COLOR_BGR2HSV);
            Mat yColorFiltered;
            inRange(imageInHSV, yLower, yUpper, yColorFiltered);
            Mat yBlurredImage;
            GaussianBlur(yColorFiltered, yBlurredImage, Size(5, 5), 0);
            Mat yEdgeImage;
            Canny(yBlurredImage, yEdgeImage, 100, 200);
            Mat yDilatedImage;
            Mat ykernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            Mat yExtraBlurMat;
            dilate(yEdgeImage, yDilatedImage, ykernel);
            GaussianBlur(yDilatedImage, yExtraBlurMat, Size(7, 7), 0);
            Mat yErodeMat;
            erode(yExtraBlurMat, yErodeMat, ykernel);

            vector<int> countOfYellowLinesAddedToEachLane;
            vector<vector<double>> yellowLaneLines;

            if (yBlurredImage.size().height > 20)
            {
                vector<Vec4i> yellowLines;
                vector<vector<Point>> yellowPointsForLines;
                HoughLinesP(yErodeMat, yellowLines, 1, CV_PI / 180, 70, 30, 10);

                for (size_t i = 0; i < yellowLines.size(); i++)
                {
                    //line(displayForLines, Point(yellowLines[i][0], yellowLines[i][1]),Point( yellowLines[i][2], yellowLines[i][3]), Scalar(0,0,255), 3, 8 );
                    if (yellowLines[i][1] > 240 || yellowLines[i][3] > 240)
                    {
                        line(displayForYellowLines, Point(yellowLines[i][0], yellowLines[i][1]), Point(yellowLines[i][2], yellowLines[i][3]), Scalar(0, 0, 255), 3, 8);
                        vector<Point> newPoint;
                        Point beginning(yellowLines[i][0], yellowLines[i][1]);
                        Point end(yellowLines[i][2], yellowLines[i][3]);
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
                        yellowPointsForLines.push_back(newPoint);
                    }
                }

                for (int j = 0; j < static_cast<int>(yellowPointsForLines.size()); j++)
                {
                    int indexOfCorrespondence = -1;
                    for (int i = 0; i < static_cast<int>(yellowLaneLines.size()); i++)
                    {
                        //averages the slope and x distance of all yellowLines in the same area
                        if (abs(yellowPointsForLines.at(j).at(0).x - yellowLaneLines.at(i).at(0)) < 300 && static_cast<int>(yellowLaneLines.size()) < 10)
                        {
                            //test
                            double oldX = yellowLaneLines.at(i).at(0);
                            double oldSlope = yellowLaneLines.at(i).at(1);
                            double newSlope = slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1));
                            double newX = yellowPointsForLines.at(j).at(0).x;
                            yellowLaneLines.at(i).at(0) = (yellowLaneLines.at(i).at(0) * countOfYellowLinesAddedToEachLane.at(i) + yellowPointsForLines.at(j).at(0).x) / (countOfYellowLinesAddedToEachLane.at(i) + 1);
                            yellowLaneLines.at(i).at(1) = (yellowLaneLines.at(i).at(1) * countOfYellowLinesAddedToEachLane.at(i) + slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1))) / (countOfYellowLinesAddedToEachLane.at(i) + 1);

                            double averageX = yellowLaneLines.at(i).at(0);
                            double averageSlope = yellowLaneLines.at(i).at(1);

                            countOfYellowLinesAddedToEachLane.at(i)++;
                            indexOfCorrespondence = j;
                        };
                    }
                    if (indexOfCorrespondence == -1)
                    {
                        // add first line when it doesnt match any of the others that already exist
                        vector<double> xPointAndSlope;
                        xPointAndSlope.push_back(yellowPointsForLines.at(j).at(0).x);
                        xPointAndSlope.push_back(slope(yellowPointsForLines.at(j).at(0), yellowPointsForLines.at(j).at(1)));
                        yellowLaneLines.push_back(xPointAndSlope);
                        countOfYellowLinesAddedToEachLane.push_back(1);
                    }
                }
            }
            Mat wColorFiltered;
            inRange(imageInHSV, wLower, wUpper, wColorFiltered);
            Mat wBlurredImage;
            GaussianBlur(wColorFiltered, wBlurredImage, Size(5, 5), 0);
            Mat wEdgeImage;
            Canny(wBlurredImage, wEdgeImage, 100, 200);
            Mat wDilatedImage;
            Mat wkernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            Mat wExtraBlurMat;
            dilate(wEdgeImage, wDilatedImage, wkernel);
            GaussianBlur(wDilatedImage, wExtraBlurMat, Size(7, 7), 0);
            Mat wErodeMat;
            erode(wExtraBlurMat, wErodeMat, wkernel);

            vector<int> countOfWhiteLinesAddedToEachLane;
            vector<vector<double>> whiteLaneLines;

            if (wBlurredImage.size().height > 20)
            {
                vector<Vec4i> whiteLines;
                vector<vector<Point>> whitePointsForLines;
                HoughLinesP(wErodeMat, whiteLines, 1, CV_PI / 180, 70, 30, 10);

                for (size_t i = 0; i < whiteLines.size(); i++)
                {
                    //line(displayForLines, Point(whiteLines[i][0], whiteLines[i][1]),Point( whiteLines[i][2], whiteLines[i][3]), Scalar(0,0,255), 3, 8 );
                    if (whiteLines[i][1] > 240 || whiteLines[i][3] > 240)
                    {
                        line(displayForWhiteLines, Point(whiteLines[i][0], whiteLines[i][1]), Point(whiteLines[i][2], whiteLines[i][3]), Scalar(0, 0, 255), 3, 8);
                        vector<Point> newPoint;
                        Point beginning(whiteLines[i][0], whiteLines[i][1]);
                        Point end(whiteLines[i][2], whiteLines[i][3]);
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
                        whitePointsForLines.push_back(newPoint);
                    }
                }

                for (int j = 0; j < static_cast<int>(whitePointsForLines.size()); j++)
                {
                    int indexOfCorrespondence = -1;
                    for (int i = 0; i < static_cast<int>(whiteLaneLines.size()); i++)
                    {
                        //averages the slope and x distance of all whiteLines in the same area
                        if (abs(whitePointsForLines.at(j).at(0).x - whiteLaneLines.at(i).at(0)) < 300 && static_cast<int>(whiteLaneLines.size()) < 10)
                        {
                            //test
                            double oldX = whiteLaneLines.at(i).at(0);
                            double oldSlope = whiteLaneLines.at(i).at(1);
                            double newSlope = slope(whitePointsForLines.at(j).at(0), whitePointsForLines.at(j).at(1));
                            double newX = whitePointsForLines.at(j).at(0).x;
                            whiteLaneLines.at(i).at(0) = (whiteLaneLines.at(i).at(0) * countOfWhiteLinesAddedToEachLane.at(i) + whitePointsForLines.at(j).at(0).x) / (countOfWhiteLinesAddedToEachLane.at(i) + 1);
                            whiteLaneLines.at(i).at(1) = (whiteLaneLines.at(i).at(1) * countOfWhiteLinesAddedToEachLane.at(i) + slope(whitePointsForLines.at(j).at(0), whitePointsForLines.at(j).at(1))) / (countOfWhiteLinesAddedToEachLane.at(i) + 1);

                            double averageX = whiteLaneLines.at(i).at(0);
                            double averageSlope = whiteLaneLines.at(i).at(1);

                            countOfWhiteLinesAddedToEachLane.at(i)++;
                            indexOfCorrespondence = j;
                        };
                    }
                    if (indexOfCorrespondence == -1)
                    {
                        // add first line when it doesnt match any of the others that already exist
                        vector<double> xPointAndSlope;
                        xPointAndSlope.push_back(whitePointsForLines.at(j).at(0).x);
                        xPointAndSlope.push_back(slope(whitePointsForLines.at(j).at(0), whitePointsForLines.at(j).at(1)));
                        whiteLaneLines.push_back(xPointAndSlope);
                        countOfWhiteLinesAddedToEachLane.push_back(1);
                    }
                }
            }
            for (int i = 0; i < static_cast<int>(yellowLaneLines.size()); i++)
            {
                ROS_INFO("Yellow X coordinate: %s", std::to_string(yellowLaneLines.at(i).at(0)).c_str());
                ROS_INFO("Yellow Slope: %s", std::to_string(yellowLaneLines.at(i).at(1)).c_str());
            }

            for (int i = 0; i < static_cast<int>(whiteLaneLines.size()); i++)
            {
                ROS_INFO("White X coordinate: %s", std::to_string(whiteLaneLines.at(i).at(0)).c_str());
                ROS_INFO("White Slope: %s", std::to_string(whiteLaneLines.at(i).at(1)).c_str());
            }

            int numLanes = static_cast<int>(whiteLaneLines.size()) + static_cast<int>(yellowLaneLines.size());
            ROS_INFO("NUMBER OF LANES FOUND: %s", std::to_string(numLanes).c_str());

            if (static_cast<int>(whiteLaneLines.size()) >= 1 && static_cast<int>(yellowLaneLines.size()) >= 1)
            {
                laneNumber = laneFinder(whiteLaneLines, yellowLaneLines, laneNumber);
            }

            ROS_INFO("Lane Number: %s", std::to_string(laneNumber).c_str());

            double laneMidPointCorrectionCoefficient = (M_PI / 6) / 640 * 1.25;

            double TURN_CONST_HARD_OUT = 3.0;
            double TURN_CONST_SOFT_OUT = 4.0;
            double TURN_CONST_HARD_IN = 2.5;
            double TURN_CONST_SOFT_IN = 3.5;
            double CENT_CORR_CONST = -0.0005;
            double SPEED_CONST = 0.75;

            if (yBlurredImage.size().height > 20 || wBlurredImage.size().height > 20)
            {
                //Right Lane
                if (static_cast<int>(yellowLaneLines.size()) >= 1 && static_cast<int>(whiteLaneLines.size()) >= 1)
                {
                    ROS_INFO("TWO LANES FOUND");
                    double wSlope = whiteLaneLines.at(0).at(1);
                    double wXCoord = whiteLaneLines.at(0).at(0);
                    double ySlope = yellowLaneLines.at(0).at(1);
                    double yXCoord = yellowLaneLines.at(0).at(0);
                    double distFromCenter = 0;
                    if(laneNumber == 0){
                        distFromCenter = 505 - (yXCoord + wXCoord) / 2;
                    } else if(laneNumber == 1){
                        distFromCenter = 505 - (wXCoord + yXCoord) / 2;
                    }
                    ROS_INFO("Center Steering Correction : %s", std::to_string(distFromCenter * CENT_CORR_CONST).c_str());
                    double difInSlope = abs(wSlope) - abs(ySlope);
                    double avgSlope = (wSlope + ySlope) / 2 + difInSlope / 2;
                    steeringAngle = 0.4 * tanh(avgSlope / TURN_CONST_HARD_OUT) + CENT_CORR_CONST * distFromCenter;
                    carSpeed = SPEED_CONST;
                }
                //Inside lanes
                else if (static_cast<int>(whiteLaneLines.size()) == 0 && static_cast<int>(yellowLaneLines.size()) >= 1)
                {
                    ROS_INFO("ONLY YELLOW LANE FOUND");
                    ///TODO: No white lane found but yellow lane found
                    //Most likely making a right turn in right la
                    double ySlope = yellowLaneLines.at(0).at(1) / 1.5;
                    double yXCoord = yellowLaneLines.at(0).at(0);
                    if (laneNumber == 0 && yXCoord <= 500)
                    {
                        ROS_INFO("LEFT LANE HARD LEFT TURN");
                        steeringAngle = 0.4 * tanh(ySlope / TURN_CONST_HARD_IN);
                        carSpeed = 0.75;
                    }
                    else if (laneNumber == 0 && yXCoord > 500)
                    {
                        ROS_INFO("LEFT LANE SOFT LEFT TURN");
                        steeringAngle = 0.4 * tanh(ySlope / TURN_CONST_SOFT_IN);
                        carSpeed = 1.0;
                    }
                    else if (laneNumber == 1 && yXCoord <= 300)
                    {
                        ROS_INFO("RIGHT LANE SOFT RIGHT TURN");
                        steeringAngle = 0.4 * tanh(ySlope / TURN_CONST_SOFT_IN);
                        carSpeed = 1.0;
                    }
                    else if (laneNumber == 1 && yXCoord > 300)
                    {
                        ROS_INFO("RIGHT LANE HARD RIGHT TURN");
                        steeringAngle = 0.4 * tanh(ySlope / TURN_CONST_HARD_IN);
                        carSpeed = 0.75;
                    }
                    else
                    {
                        ROS_INFO("Can't Determine Steering Angle");
                        carSpeed = 0.75;
                    }

                    if(abs(ySlope) < 1.5 && laneNumber == 0){
                        steeringAngle += CENT_CORR_CONST * (820 - yXCoord);
                    } else if(abs(ySlope) < 1.5 && laneNumber == 1){
                        steeringAngle += CENT_CORR_CONST * (188 - yXCoord);
                    }
                    carSpeed = SPEED_CONST;
                    
                }
                //Outside Lanes
                else if (static_cast<int>(yellowLaneLines.size()) == 0 && static_cast<int>(whiteLaneLines.size()) >= 1)
                {
                    ROS_INFO("ONLY WHITE LANE FOUND");
                    ///TODO: No yelow lane found but white lane found
                    //Most likely making a left turn in the right lane
                    double wSlope = whiteLaneLines.at(0).at(1) / 1.5;
                    double wXCoord = whiteLaneLines.at(0).at(0);
                    if (laneNumber == 0 && wXCoord >= 500)
                    {
                        ROS_INFO("LEFT LANE HARD RIGHT TURN");
                        steeringAngle = 0.4 * tanh(wSlope / TURN_CONST_HARD_OUT);
                        carSpeed = 0.75;
                    }
                    else if (laneNumber == 0 && wXCoord < 500)
                    {
                        ROS_INFO("LEFT LANE SOFT RIGHT TURN");
                        steeringAngle = 0.4 * tanh(wSlope / TURN_CONST_SOFT_OUT);
                        carSpeed = 1.0;
                    }
                    else if (laneNumber == 1 && wXCoord >= 625)
                    {
                        ROS_INFO("RIGHT LANE SOFT LEFT TURN");
                        steeringAngle = 0.4 * tanh(wSlope / TURN_CONST_SOFT_OUT);
                        carSpeed = 1.0;
                    }
                    else if (laneNumber == 1 && wXCoord < 625)
                    {
                        ROS_INFO("RIGHT LANE HARD LEFT TURN");
                        steeringAngle = 0.4 * tanh(wSlope / TURN_CONST_HARD_OUT);
                        carSpeed = 0.75;
                    }
                    else
                    {
                        ROS_INFO("Can't Determine Steering Angle");
                        carSpeed = 0.75;
                    }

                    if(abs(wSlope) < 1.5 && laneNumber == 1){
                        steeringAngle += CENT_CORR_CONST * (820 - wXCoord);
                    } else if(abs(wSlope) < 1.5 && laneNumber == 0){
                        steeringAngle += CENT_CORR_CONST * (188 - wXCoord);
                    }
                    carSpeed = SPEED_CONST;
                }
                else
                {
                    ROS_INFO("No lane lines found");
                    if (laneNumber == 0)
                    {
                        ROS_INFO("LEFT LANE, TURNING RIGHT TO REAQUIRE LANES");
                        steeringAngle = 0.2;
                    }
                    else
                    {
                        ROS_INFO("RIGHT LANE, TURNING LEFT TO REAQUIRE LANES");
                        steeringAngle = -0.2;
                    }
                    carSpeed = SPEED_CONST;
                }

                

                // if(abs(steeringAngle) < 0.1){
                //     carSpeed = 2.0;
                // } else if(abs(steeringAngle) < 0.25){
                //     carSpeed = 1.5;
                // } else if(abs(steeringAngle) < 0.35){
                //     carSpeed = 1.0;
                // } else{
                //     carSpeed = 0.75;
                // }
            }

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
        //UPLOAD DRIVE DATA
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
        drive_msg.steering_angle = steeringAngle * -0.8;
        drive_msg.speed = carSpeed;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
        //
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

    int
    laneFinder(vector<vector<double>> wLines, vector<vector<double>> yLines, int lane)
    {
        int laneFind = lane;
        if (laneFind > -2)
        {
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
