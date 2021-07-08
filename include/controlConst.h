/**
 * 
 * @author Will Anderson
 * 
 * This file handels the steering calculation based on which lane lines are found and where in the image they are
 * I have used a hyperbolic tangent function to control the steering of the vehicle since this allows for maxing out the angle at 0.4, the max steering angle of the car
 * 
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
using namespace std;
using namespace cv;
double slope(Point first, Point second);

class ControlConst
{
public:
    static vector<double> steerSpeed(vector<vector<double>> yellowLaneLines, vector<vector<double>> whiteLaneLines, int laneNumber)
    {
        double steeringAngle;

        /** -------------------------------------------------**\
        * ------------------Define Constants------------------ *
        \**--------------------------------------------------**/
        double carSpeed;
        double laneMidPointCorrectionCoefficient = (M_PI / 6) / 640 * 1.25;
        double TURN_CONST_HARD_OUT = 3.0;
        double TURN_CONST_SOFT_OUT = 4.0;
        double TURN_CONST_HARD_IN = 2.5;
        double TURN_CONST_SOFT_IN = 3.5;
        double CENT_CORR_CONST = -0.0005;
        double SPEED_CONST = 0.75;

        /** -------------------------------------------------**\
        * -------------YELLOW & WHITE LANES FOUND------------- *
        \**--------------------------------------------------**/
        if (static_cast<int>(yellowLaneLines.size()) >= 1 && static_cast<int>(whiteLaneLines.size()) >= 1)
        {
            ROS_INFO("TWO LANES FOUND");
            //Get solpes and intercepts from vectors
            double wSlope = whiteLaneLines.at(0).at(1); //Slope of the white line
            double wXCoord = whiteLaneLines.at(0).at(0); //X coordinate of the white line (at the bottom of the image)
            double ySlope = yellowLaneLines.at(0).at(1); //Slope of the yellow line
            double yXCoord = yellowLaneLines.at(0).at(0); //X coordinate of the yellow line (at the bottom of the image)
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

            //Need these two calculations as the camera is not centered on the car
            //This allows for average slope to account for the non-centered image and calculate steering angle from there
            double difInSlope = abs(wSlope) - abs(ySlope);
            double avgSlope = (wSlope + ySlope) / 2 + difInSlope / 2;
            //Using hyperbolic tangent * the max steering angle to find the necesary steering angle
            steeringAngle = 0.4 * tanh(avgSlope / TURN_CONST_HARD_OUT) + CENT_CORR_CONST * distFromCenter;
            carSpeed = SPEED_CONST;
        }
        /** -------------------------------------------------**\
        * ---------------CONTROL FOR INNER LANES-------------- *
        \**--------------------------------------------------**/
        else if (static_cast<int>(whiteLaneLines.size()) == 0 && static_cast<int>(yellowLaneLines.size()) >= 1)
        {
            ROS_INFO("ONLY YELLOW LANE FOUND");
            ///No white lane found but yellow lane found
            //Most likely making a right turn in right la
            double ySlope = yellowLaneLines.at(0).at(1) / 1.5; //This /1.5 just works since we do not have 2 slopes to average out
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

            //Center correction control once the car is almopst fully around the corner (when the slope returns to less than 1.5)
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
        /** -------------------------------------------------**\
        * ---------------CONTROL FOR OUTER LANES-------------- *
        \**--------------------------------------------------**/
        else if (static_cast<int>(yellowLaneLines.size()) == 0 && static_cast<int>(whiteLaneLines.size()) >= 1)
        {
            ROS_INFO("ONLY WHITE LANE FOUND");
            ///No yelow lane found but white lane found
            //Most likely making a left turn in the right lane
            double wSlope = whiteLaneLines.at(0).at(1) / 1.5; //This /1.5 just works since we do not have 2 slopes to average out
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

            //Center correction control once the car is almopst fully around the corner (when the slope returns to less than 1.5)
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
        /** -------------------------------------------------**\
        * -------------------NO LANES FOUND------------------- *
        \**--------------------------------------------------**/
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
};