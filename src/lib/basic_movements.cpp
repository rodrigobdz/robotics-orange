#ifndef BASIC_MOVEMENTS_LIB
#define BASIC_MOVEMENTS_LIB

// Import pi constant (M_PI)
#define _USE_MATH_DEFINES
#include <cmath>

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/ResetEncoders.h"



class BasicMovements
{
    public:
        BasicMovements()
        {
            // Set up encoders callback
            encoderSubscriber   = n.subscribe("sensor_packet", 1, &BasicMovements::encoderCallback, this);
            diffDriveClient     = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

            // Set up laser callback
            laserSubscriber     = n.subscribe("scan_filtered", 1, &BasicMovements::laserCallback, this);

            // Set up reset encoders client
            resetEncodersClient = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
        }

        void stop();
        bool drive(float distanceInMeters, float speed = MEDIUM_SPEED);
        bool rotate(float angleInDegrees, float speed = MEDIUM_SPEED);
        bool rotateAbs(float angleInDegrees, float speed = MEDIUM_SPEED);

    private:
        static const float MAXIMUM_SPEED         = 10;
        static const float MEDIUM_SPEED          = 5;
        static const float SLOW_SPEED            = 1;
        static const float ONE_METER_IN_RAD      = 30.798;
        static const float NINETY_DEGREES_IN_RAD = 30.798 * 0.196349; // 2pir^2 / 4, r = 0.26/2
        // Distances are given in meters
        static const float SAFETY_DIS            = 0.15; // Minimum distance to keep when driving
        static const float CELL_LENGTH           = 0.80;
        static const bool DEBUG                  = true; // Defines if output should be printed
        static const bool CALLBACK_DEBUG         = false; // Decide to print output from callbacks
        static const int LOOP_RATE               = 16; // Used for loop rate


        ros::NodeHandle n;
        // Encoders
        ros::Subscriber encoderSubscriber;
        // Differential Drive
        create_fundamentals::DiffDrive diffDriveService;
        ros::ServiceClient diffDriveClient;
        // Reset Encoders
        create_fundamentals::ResetEncoders resetEncodersService;
        ros::ServiceClient resetEncodersClient;
        // Laser
        ros::Subscriber laserSubscriber;


        std::vector<float> ranges;
        float leftEncoder, rightEncoder;


        void encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void resetEncoders();
};



void BasicMovements::stop()
{
    if(DEBUG) {
        ROS_INFO("diffDrive 0 0");
    }

    diffDriveService.request.left  = 0;
    diffDriveService.request.right = 0;
    diffDriveClient.call(diffDriveService);
}

/*
 * Params: distance in meter. If negative distance robot
 *         will go backwards
 *         No negative speed allowed.
 * Returns: false if obstacle was found otherwise true
**/
bool BasicMovements::drive(float distanceInMeters, float speed)
{
    // TODO: Modify for variable distance
    // TODO: Check in laser callback if object is on the way to stop in that case

    if(DEBUG) {
        ROS_INFO("diffDrive %f %f distance: %f m", speed, speed, distanceInMeters);
    }

    speed = fabs(speed);
    float sign              = distanceInMeters < 0 ? -1 : 1; // Check if speed positive or negative
    float distanceInRadians = distanceInMeters * ONE_METER_IN_RAD;
    float threshold         = fabs(distanceInRadians) - (fabs(distanceInRadians) * 0.025);

    resetEncoders();
    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()) {
        if (speed == 0) {
            break;
        }

        if ((sign*leftEncoder) >= threshold || (sign*rightEncoder) >= threshold) {
            speed = 0;
        }

        diffDriveService.request.left  = sign * speed;
        diffDriveService.request.right = sign * speed;
        diffDriveClient.call(diffDriveService);

        ros::spinOnce();
        loop_rate.sleep();
    }

    resetEncoders();

    return true;
}

/* Rotate the robot corresponding to its local coordinate system.
   That means the robots face lays is exactly 90 degrees and its
   righthandside is 0 degrees. So if you 30 degrees to this
   function, the robot will rotate 60 degrees to the right. */
bool BasicMovements::rotateAbs(float angleInDegrees, float speed)
{
    if (angleInDegrees > 180)
        angleInDegrees = 180;

    if (angleInDegrees < 0)
        angleInDegrees = 0;

    return rotate(angleInDegrees-90, speed);
}

/*
 * Params: if angle positive robot will rotate clockwise
 *         else if negative counter clockwise
 *         No negative speed allowed.
 * Returns: false if obstacle was found otherwise true
**/
bool BasicMovements::rotate(float angleInDegrees, float speed)
{
    if(DEBUG) {
        ROS_INFO("diffDrive %f %f angle: %fº", speed, speed, angleInDegrees);
    }

    speed = fabs(speed);
    float sign           = angleInDegrees < 0 ? -1 : 1;
    float angleInRadians = angleInDegrees * (NINETY_DEGREES_IN_RAD / 90);
    float threshold      = fabs(angleInRadians) - (fabs(angleInRadians) * 0.04);

    resetEncoders();
    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()){
        if (speed == 0) {
            break;
        }

        if ((sign*leftEncoder) >= threshold || (sign*rightEncoder) >= threshold) {
            speed = 0;
        }

        diffDriveService.request.left  = sign * -speed;
        diffDriveService.request.right = sign * speed;
        diffDriveClient.call(diffDriveService);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}



/********************** HELPERS *****************************/



void BasicMovements::encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    if(CALLBACK_DEBUG) {
        ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
    }
    leftEncoder  = msg->encoderLeft;
    rightEncoder = msg->encoderRight;
}

void BasicMovements::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
}

/*
 * Reset the encoders and corresponding helper values.
**/
void BasicMovements::resetEncoders()
{
    resetEncodersClient.call(resetEncodersService);
    leftEncoder = rightEncoder = 0;
}

#endif // BASIC_MOVEMENTS_LIB