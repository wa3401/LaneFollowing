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

using namespace std;

class ImageProcessor
{

public:
    Mat getErode(Mat yBlurredImage)
    {
        Mat yEdgeImage;
        Canny(yBlurredImage, yEdgeImage, 100, 200);
        Mat yDilatedImage;
        Mat ykernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        Mat yExtraBlurMat;
        dilate(yEdgeImage, yDilatedImage, ykernel);
        GaussianBlur(yDilatedImage, yExtraBlurMat, Size(7, 7), 0);
        Mat yErodeMat;
        erode(yExtraBlurMat, yErodeMat, ykernel);

        return yErodeMat;
    }
    Mat getBlur(Scalar yLower, Scalar yUpper, Mat croppedImage)
    {

        Mat yuv_img;
        cvtColor(croppedImage, yuv_img, CV_BGR2YUV);
        std::vector<Mat> channels;
        split(yuv_img, channels);
        equalizeHist(channels[0], channels[0]);
        merge(channels, yuv_img);
        cvtColor(yuv_img, croppedImage, CV_YUV2BGR);
        //Yellow Line Detection
        Mat imageInHSV;
        cvtColor(croppedImage, imageInHSV, COLOR_BGR2HSV);
        Mat yColorFiltered;
        inRange(imageInHSV, yLower, yUpper, yColorFiltered);
        Mat yBlurredImage;
        GaussianBlur(yColorFiltered, yBlurredImage, Size(5, 5), 0);
        return yBlurredImage;
    }
    vector<vector<double>> processImage(Mat yErodeMat, Mat yBlurredImage)
    {
        Mat displayForYellowLines(270, 960, CV_8UC3, Scalar(0, 0, 0));

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
        return yellowLaneLines;
    }
};