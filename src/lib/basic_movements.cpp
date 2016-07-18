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
#include <wall_recognition.cpp>
#include <math.h>

class BasicMovements
{
    public:
        BasicMovements()
        {
            // Set up encoders callback
            encoderSubscriber = n.subscribe("sensor_packet", 1, &BasicMovements::encoderCallback, this);
            diffDriveClient = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

            // Set up laser callback
            laserSubscriber = n.subscribe("scan_filtered", 1, &BasicMovements::laserCallback, this);

            // Set up reset encoders client
            resetEncodersClient = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

            // Initialize minimum range to a default value to be able to stop when obstacle is found
            laserInitialized = false;
            encoderInitialized = false;
        }

        void stop();
        bool drive(float distanceInMeters, float speed = DEFAULT_SPEED);
        bool rotate(float angleInDegrees, float speed = DEFAULT_SPEED);
        bool rotateLeft(float speed = DEFAULT_SPEED);
        bool rotateRight(float speed = DEFAULT_SPEED);
        bool rotateBackwards(float speed = DEFAULT_SPEED);

        bool driveWall(float distanceInMeters, float speed = DEFAULT_SPEED);
        bool turnLeft();
        bool turnRight();

        bool move(float desiredVelocity, float desiredTurningVelocity);

    private:
        static const bool DETECT_OBSTACLES = true;
        static const bool DEBUG = false;          // Defines if output should be printed
        static const bool CALLBACK_DEBUG = false; // Decide to print output from callbacks
        float minimumRange;                       // Global variable to store minimum distance to object if found
        bool laserInitialized;
        bool encoderInitialized;

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

        // WallRecognition
        WallRecognition wall_recognition;

        std::vector<float> ranges;
        float leftEncoder, rightEncoder;

        bool detectObstacles(float sign);
        bool turn(int direction);
        void encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg);
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void initialiseEncoder();
        void resetEncoders();
};

bool BasicMovements::move(float desiredVelocity, float desiredTurningVelocity)
{
    float vLeft = 1 / RAD_RADIUS * (desiredVelocity + ROB_BASE / 2 * desiredTurningVelocity);
    float vRight = 1 / RAD_RADIUS * (desiredVelocity - ROB_BASE / 2 * desiredTurningVelocity);

    diffDriveService.request.left = vLeft;
    diffDriveService.request.right = vRight;

    diffDriveClient.call(diffDriveService);
}

void BasicMovements::stop()
{
    if (DEBUG) {
        ROS_INFO("STOP");
    }

    diffDriveService.request.left = 0;
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
    if (DEBUG) {
        ROS_INFO("diffDrive %f %f distance: %f m", speed, speed, distanceInMeters);
    }

    speed = fabs(speed);
    float sign = distanceInMeters < 0 ? -1 : 1; // Check if speed positive or negative
    float distanceInRadians = distanceInMeters * ONE_METER_IN_RAD;
    float threshold = fabs(distanceInRadians) - (fabs(distanceInRadians) * 0.025);

    resetEncoders();

    while (ros::ok()) {
        ros::spinOnce();
        if(DETECT_OBSTACLES) {
            if(!detectObstacles(sign)) {
                return false;
            }
        }

        if ((sign * leftEncoder) >= threshold || (sign * rightEncoder) >= threshold) {
            BasicMovements::stop();
            return true;
        }

        diffDriveService.request.left = sign * speed;
        diffDriveService.request.right = sign * speed;
        diffDriveClient.call(diffDriveService);

        ros::spinOnce();
    }

    return false;
}

/*
 * Drives straight forward keeping distance to the wall and at the
 * same time aligning to it.
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::driveWall(float distanceInMeters, float speed)
{
    // Use normal drive function when distance is negative
    if (distanceInMeters < 0) {
        return drive(distanceInMeters, speed);
    }

    initialiseEncoder();

    float wishLeftEncoder = leftEncoder + distanceInMeters / RAD_RADIUS;
    float wishRightEncoder = rightEncoder + distanceInMeters / RAD_RADIUS;

    if (DEBUG) {
        ROS_INFO("leftEncoder = %f, wishLeftEncoder = %f", leftEncoder, wishLeftEncoder);
        ROS_INFO("rightEncoder = %f, wishRightEncoder = %f", rightEncoder, wishRightEncoder);
    }

    float vLeft = 0;
    float vRight = 0;

    bool odometryCondition = true;
    while (odometryCondition) {
        ros::spinOnce();

        if (DETECT_OBSTACLES) {
            if (!detectObstacles(1)) {
                return false;
            }
        }

        std::vector<Wall*> walls = wall_recognition.getWalls();
        Wall* frontWall = wall_recognition.getFrontWall(walls);
        Wall* rightWall = wall_recognition.getRightWall(walls);
        Wall* leftWall = wall_recognition.getLeftWall(walls);

        if (CALLBACK_DEBUG) {
            ROS_INFO("Distance to drive %f", fabs(wishRightEncoder - rightEncoder) * RAD_RADIUS);
        }

        if (walls.size() == 0 || walls.size() == 1 && frontWall != NULL) { // Drive forward
            vLeft = DEFAULT_SPEED;
            vRight = DEFAULT_SPEED;
        } else if (rightWall != NULL || leftWall != NULL) {
            // Search for nearest wall
            Wall* nearestWall = wall_recognition.getNearestSideWall(walls);
            float wallAngle = nearestWall->getAngleInRadians();
            float wallDistance = nearestWall->getDistanceInMeters();

            float maximalTurnAngle = PI / 2;
            float slopeOfFunction = maximalTurnAngle / (CELL_LENGTH / 2);
            float distanceCorrection = slopeOfFunction * wallDistance - maximalTurnAngle;

            float angleCorrection = (-(PI / 4) * tan(2 * wallAngle));


            if (nearestWall->isLeftWall()) {
                distanceCorrection = -distanceCorrection;
            }

            vLeft = speed + 1 / RAD_RADIUS * (angleCorrection + distanceCorrection) * ROB_BASE / 2;
            vRight = speed - 1 / RAD_RADIUS * (angleCorrection + distanceCorrection) * ROB_BASE / 2;

        }

        if (frontWall != NULL && wishRightEncoder - 0.1 < rightEncoder) {
            // ROS_INFO("frontwall distane = %f", frontWall->getDistanceInMeters());
            if (frontWall->getDistanceInMeters() > 0.35 && frontWall->getDistanceInMeters() < 0.7) {
                if(frontWall->getDistanceInMeters() > 0.40){
                    vLeft = 10 * frontWall->getDistanceInMeters() - 2;
                    vRight = 10 * frontWall->getDistanceInMeters() - 2;
                    odometryCondition = true;
                } else {
                    vLeft = 2;
                    vRight = 2;
                    odometryCondition = false;
                }
            } else if (frontWall->getDistanceInMeters() > 0.75 && frontWall->getDistanceInMeters() < 1.1) {
                if(frontWall->getDistanceInMeters() > 0.80){
                    vLeft = 10 * frontWall->getDistanceInMeters() - 6;
                    vRight = 10 * frontWall->getDistanceInMeters() - 6;
                    odometryCondition = true;
                } else {
                    vLeft = 2;
                    vRight = 2;
                    odometryCondition = false;
                }
            } else {
                odometryCondition = false;
            }
        } else if (frontWall != NULL) {
            odometryCondition = true;
        } else {
            odometryCondition = fabs(wishRightEncoder - rightEncoder) > 1;
        }

        diffDriveService.request.left = vLeft;
        diffDriveService.request.right = vRight;

        diffDriveClient.call(diffDriveService);
    }

    return true;
}

/*
 * Params: if angle positive robot will rotate clockwise
 *         else if negative counter clockwise
 *         No negative speed allowed.
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::rotate(float angleInDegrees, float speed)
{
    // Variables
    float angleInRadians = angleInDegrees * (PI / 2) / 90;
    float threshold = NINETY_DEGREES_IN_RAD / 10;

    initialiseEncoder();

    float wishLeftEncoder = leftEncoder - 1 / RAD_RADIUS * (ROB_BASE / 2 * angleInRadians);
    float wishRightEncoder = rightEncoder + 1 / RAD_RADIUS * (ROB_BASE / 2 * angleInRadians);

    while (ros::ok()) {
        ros::spinOnce();

        if(CALLBACK_DEBUG) {
            ROS_INFO("leftEncoder %f, wishLeftEncoder %f", leftEncoder, wishLeftEncoder);
            ROS_INFO("fabs((wishLeftEncoder - leftEncoder)) = %f", fabs((wishLeftEncoder - leftEncoder)));
        }

        if (fabs((wishLeftEncoder - leftEncoder)) < 0.1) {
            if (DEBUG) {
                ROS_INFO("Perfect Angle");
            }
            stop();
            return true;
        }

        if (wishLeftEncoder > leftEncoder) {
            move(0, 1);
        } else {
            move(0, -1);
        }
    }

    return false;
}

/*
 * Parameter: rotation speed
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::rotateLeft(float speed) { return rotate(90, speed); }

/*
 * Parameter: rotation speed
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::rotateRight(float speed) { return rotate(-90, speed); }

/*
 * Parameter: rotation speed
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::rotateBackwards(float speed) { return rotate(180, speed); }

/*
 * Turns left around a corner. This is not the same as rotate left.
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::turnLeft() { return turn(LEFT); }

/*
 * Turns right around a corner. This is not the same as rotate right.
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::turnRight() { return turn(RIGHT); }

/*
 * Turns left or right around a corner. This is not the same as rotate left or right.
 *
 * Parameter: direction in which turn should be executed.
 *            direction should be left or right.
 *            If invalid direction is passed over then right will be executed.
 *
 * Returns: false if obstacle was found otherwise true
 **/
bool BasicMovements::turn(int direction)
{
    // Initialize default turning direction to right.
    // Float type is used to avoid data loss when multiplying
    // with other floats
    float sign = 1;
    if (direction == LEFT) {
        sign = -1;
    }
    float fixRadius = 0.05;

    initialiseEncoder();

    float wishLeftEncoder =
        leftEncoder + (1 / (RAD_RADIUS * 2)) * ((CELL_LENGTH + fixRadius) - ROB_BASE / 2 + ROB_BASE / 2 * sign * -PI / 2);
    float wishRightEncoder =
        rightEncoder + (1 / (RAD_RADIUS * 2)) * ((CELL_LENGTH + fixRadius) - ROB_BASE / 2 - ROB_BASE / 2 * sign * -PI / 2);

    while (ros::ok()) {
        ros::spinOnce();
        if(DETECT_OBSTACLES) {
            if(!detectObstacles(1)){
                return false;
            }
        }

        if(CALLBACK_DEBUG) {
            ROS_INFO("rightEncoder %f, wishRightEncoder %f", rightEncoder, wishRightEncoder);
        }

        // Check if robot rotation is enough including error margin
        if (direction == RIGHT && wishRightEncoder < rightEncoder) {
            // stop();
            return true;
        }
        if (direction == LEFT && wishLeftEncoder < leftEncoder) {
            // stop();
            return true;
        }


        // Left = 7.556252, Right = 15.869389
        // float maxSpeedFactor = 0.9452159752 * 2;
        float maxSpeedFactor = 1;
        move(maxSpeedFactor * ((CELL_LENGTH + fixRadius) - ROB_BASE / 2) / 3, maxSpeedFactor * sign * PI / 2 / 3);
    }

    return false;
}

bool BasicMovements::detectObstacles(float sign)
{
    if(DEBUG) {
        ROS_INFO("detectObstacles");
    }
    initialiseEncoder();
    // Check if robot is about to crash into something
    // only when robot has to drive forward
    if (minimumRange < SAFETY_DISTANCE && sign > 0) {
        std::vector<Wall*> walls = wall_recognition.getWalls();
        if(wall_recognition.hasFrontWall(walls)){
            Wall* frontWall = wall_recognition.getFrontWall(walls);
            if (frontWall->isConfirmed() && frontWall->getDistanceInMeters() < SAFETY_DISTANCE) {
                // Robot recognized an obstacle, distance could not be completed
                if(DEBUG){
                    ROS_INFO("Drive: Obstacle obstructing");
                }
                stop();
                return false;
            }
        }
    }
    return true;
}


/********************** HELPERS *****************************/

void BasicMovements::encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    if (CALLBACK_DEBUG) {
        ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
    }

    encoderInitialized = true;
    leftEncoder = msg->encoderLeft;
    rightEncoder = msg->encoderRight;
}

void BasicMovements::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserInitialized = true;
    ranges = msg->ranges;

    // Initialize minimum range to a default value
    minimumRange = LASER_MAX_REACH;
    // Find minimum in ranges
    for (int i = 0; i < LASER_COUNT - 1; ++i) {
        if (ranges[i] < minimumRange) {
            // Local minimum was found
            minimumRange = ranges[i];
        }
    }
}

void BasicMovements::initialiseEncoder()
{
    ros::spinOnce();
    while (!encoderInitialized) {
        // Get laser data before driving to recognize obstacles beforehand
        ros::spinOnce();
    }
}

/*
 * Reset the encoders and corresponding helper values.
 **/
void BasicMovements::resetEncoders()
{
    resetEncodersClient.call(resetEncodersService);
    leftEncoder = rightEncoder = 0;
    encoderInitialized = false;
}

#endif // BASIC_MOVEMENTS_LIB
