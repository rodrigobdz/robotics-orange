#ifndef BASIC_MOVEMENTS_LIB
#define BASIC_MOVEMENTS_LIB

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/ResetEncoders.h"
#include <constants.cpp>

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
            
            // Initialize minimum range to a default value to be able to stop when obstacle is found
            minimumRange        = 123;
        }

        void stop();
        bool drive(float distanceInMeters, float speed   = DEFAULT_SPEED);
        bool rotate(float angleInDegrees, float speed    = DEFAULT_SPEED);
        bool rotateAbs(float angleInDegrees, float speed = DEFAULT_SPEED);

    private:
        static const float DEFAULT_SPEED   = 7;
        static const bool DETECT_OBSTACLES = true;
        static const bool DEBUG            = false; // Defines if output should be printed
        static const bool CALLBACK_DEBUG   = false; // Decide to print output from callbacks
        float minimumRange; // Global variable to store minimum distance to object if found

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
        ROS_INFO("STOP diffDrive 0 0");
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
        if (DETECT_OBSTACLES) {
            // Get laser data before driving to recognize obstacles beforehand
            ros::spinOnce();
            // Check if minimumRange variable was already initialized by laser callback
            if (minimumRange == 123) {
                // Ranges are not initialized
                // Sleep and continue loop
                loop_rate.sleep();
                continue;
            }

            // Check if robot is about to crash into something
            if (minimumRange < SAFETY_DIS) {
                // Robot recognized an obstacle, distance could not be completed
                BasicMovements::stop();
                return false;
            }
        }

        if ((sign*leftEncoder) >= threshold || (sign*rightEncoder) >= threshold) {
            BasicMovements::stop();
            break;
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
    if(fabs(angleInDegrees) < 5) return true;

    speed = fabs(speed);
    float sign           = angleInDegrees < 0 ? -1 : 1;
    float angleInRadians = angleInDegrees * (NINETY_DEGREES_IN_RAD / 90);
    float threshold      = fabs(angleInRadians) - (NINETY_DEGREES_IN_RAD / 90)/2;

    resetEncoders();
    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()){
        if (speed == 0) {
            break;
        }

        if (fabs(leftEncoder) >= threshold || fabs(rightEncoder) >= threshold) {
            speed = 0;
        }

        if(DEBUG) {
            ROS_INFO("diffDrive %f %f angle: %fº", sign * -speed, sign * speed, angleInDegrees);
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

    // Initialize minimum range to a default value
    minimumRange = LASER_MAX_REACH;
    // Find minimum in ranges
    for (int i = 0; i < LASER_COUNT-1; ++i) {
        if (ranges[i] < minimumRange) {
            // Local minimum was found
            minimumRange = ranges[i];
        }
    }
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