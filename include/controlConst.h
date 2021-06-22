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

class ControlConst
{
public:
    static vector<double> steerSpeed(vector<vector<double>> yellowLaneLines, vector<vector<double>> whiteLaneLines, int laneNumber)
    {
        double steeringAngle;
        double carSpeed;
        double laneMidPointCorrectionCoefficient = (M_PI / 6) / 640 * 1.25;

        double TURN_CONST_HARD_OUT = 3.0;
        double TURN_CONST_SOFT_OUT = 4.0;
        double TURN_CONST_HARD_IN = 2.5;
        double TURN_CONST_SOFT_IN = 3.5;
        double CENT_CORR_CONST = -0.0005;
        double SPEED_CONST = 0.75;

        //Right Lane
        if (static_cast<int>(yellowLaneLines.size()) >= 1 && static_cast<int>(whiteLaneLines.size()) >= 1)
        {
            ROS_INFO("TWO LANES FOUND");
            double wSlope = whiteLaneLines.at(0).at(1);
            double wXCoord = whiteLaneLines.at(0).at(0);
            double ySlope = yellowLaneLines.at(0).at(1);
            double yXCoord = yellowLaneLines.at(0).at(0);
            double distFromCenter = 0;
            if (laneNumber == 0)
            {
                distFromCenter = 462 - (yXCoord + wXCoord) / 2;
            }
            else if (laneNumber == 1)
            {
                distFromCenter = 462 - (wXCoord + yXCoord) / 2;
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

            if (abs(ySlope) < 1.5 && laneNumber == 0)
            {
                steeringAngle += CENT_CORR_CONST * (820 - yXCoord);
            }
            else if (abs(ySlope) < 1.5 && laneNumber == 1)
            {
                steeringAngle += CENT_CORR_CONST * (125 - yXCoord);
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

            if (abs(wSlope) < 1.5 && laneNumber == 1)
            {
                steeringAngle += CENT_CORR_CONST * (820 - wXCoord);
            }
            else if (abs(wSlope) < 1.5 && laneNumber == 0)
            {
                steeringAngle += CENT_CORR_CONST * (125 - wXCoord);
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

        vector<double> speedSteer;
        speedSteer.push_back(carSpeed);
        speedSteer.push_back(steeringAngle);

        return speedSteer;
    }

    static int
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