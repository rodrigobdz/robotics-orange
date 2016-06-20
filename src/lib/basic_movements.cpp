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
#include <ransac.cpp>
#include <math.h>

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
        laserInitialized    = false;
        encoderInitialized  = false;
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

  private:
    static const bool DETECT_OBSTACLES   = true;
    static const bool DEBUG              = false; // Defines if output should be printed
    static const bool CALLBACK_DEBUG     = false; // Decide to print output from callbacks
    float minimumRange;                           // Global variable to store minimum distance to object if found
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

    // Ransac
    Ransac ransac;

    std::vector<float> ranges;
    float leftEncoder, rightEncoder;

    bool move(float desiredVelocity, float desiredTurningVelocity);
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
    float sign              = distanceInMeters < 0 ? -1 : 1; // Check if speed positive or negative
    float distanceInRadians = distanceInMeters * ONE_METER_IN_RAD;
    float threshold         = fabs(distanceInRadians) - (fabs(distanceInRadians) * 0.025);

    resetEncoders();

    while (ros::ok()) {
        if (DETECT_OBSTACLES) {
            while (!laserInitialized) {
                // Get laser data before driving to recognize obstacles beforehand
                ros::spinOnce();
            }

            // Check if robot is about to crash into something
            if (minimumRange < SAFETY_DISTANCE) {
                // Robot recognized an obstacle, distance could not be completed
                stop();
                return false;
            }
        }

        if ((sign * leftEncoder) >= threshold || (sign * rightEncoder) >= threshold) {
            BasicMovements::stop();
            return true;
        }

        diffDriveService.request.left  = sign * speed;
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
    initialiseEncoder();

    float wishLeftEncoder  = leftEncoder + distanceInMeters / RAD_RADIUS;
    float wishRightEncoder = rightEncoder + distanceInMeters / RAD_RADIUS;

    std::vector<Wall*> walls;

    while (fabs(wishRightEncoder - rightEncoder) > 1) {
        ros::spinOnce();
        walls = ransac.getWalls();

        if (walls.size() == 0) { // Drive forward
            move(0.2, 0);
        } else {

            // Search for nearest wall
            float smallestDistance = 100;
            Wall nearestWall(0,0);

            for (int i = 0; i < walls.size(); i++) {
                if (smallestDistance > walls[i]->getDistanceInMeters()) {
                    nearestWall = *walls[i];
                    smallestDistance = walls[i]->getDistanceInMeters();
                }
            }

            float wallAngle = nearestWall.getAngleInRadians();
            float wallDistance = nearestWall.getDistanceInMeters();

            float distanceCorrectur;
            float angleCorrectur;

            if (nearestWall.isLeftWall()) {
                distanceCorrectur = (0.625 * PI) * wallDistance - PI / 4;
                angleCorrectur = -tan(2 * wallAngle) * PI / 4;
            } else if (nearestWall.isRightWall()) {
                distanceCorrectur = -(0.625 * PI) * wallDistance + PI / 4;
                angleCorrectur = -tan(2 * wallAngle) * PI / 4;
            } else {
                if (nearestWall.isFrontWall() && wallDistance < SAFETY_DISTANCE) {
                    // Robot recognized an obstacle, distance could not be completed
                    stop();
                    return false;
                }
            }
            // Just angle
            // float vLeft = 1 / RAD_RADIUS * (angleCorrectur * ROB_BASE / 2);
            // float vRight = 1 / RAD_RADIUS * (-angleCorrectur * ROB_BASE / 2);
            // Just distance
            // float vLeft = 1 / RAD_RADIUS * (-distanceCorrectur * ROB_BASE / 2);
            // float vRight = 1 / RAD_RADIUS * (distanceCorrectur * ROB_BASE / 2);

            float vLeft = 1 / RAD_RADIUS * (0.2 + (angleCorrectur - distanceCorrectur) * ROB_BASE / 2);
            float vRight = 1 / RAD_RADIUS * (0.2 + (-angleCorrectur + distanceCorrectur) * ROB_BASE / 2);

            // ROS_INFO("distanceCorrectur = %f, angleCorrectur = %f", distanceCorrectur, angleCorrectur);
            // ROS_INFO("vLeft = %f, vRight = %f", vLeft, vRight);

            diffDriveService.request.left  = vLeft;
            diffDriveService.request.right = vRight;

            diffDriveClient.call(diffDriveService);
        }
    }
    move(0, 0);

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
    float angleInRadians   = angleInDegrees * (PI / 2) / 90;
    float threshold        = NINETY_DEGREES_IN_RAD / 10;

    initialiseEncoder();

    float wishLeftEncoder  = leftEncoder - 1 / RAD_RADIUS * (ROB_BASE / 2 * angleInRadians);
    float wishRightEncoder = rightEncoder + 1 / RAD_RADIUS * (ROB_BASE / 2 * angleInRadians);

    while (ros::ok()) {
        ros::spinOnce();
        if(DEBUG) {
            // ROS_INFO("leftEncoder %f, wishLeftEncoder %f", leftEncoder, wishLeftEncoder);
            ROS_INFO("fabs((wishLeftEncoder - leftEncoder)) = %f", fabs((wishLeftEncoder - leftEncoder)));
        }

        if (fabs((wishLeftEncoder - leftEncoder)) < 0.1) {
            if(DEBUG) {
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

bool BasicMovements::rotateLeft(float speed = DEFAULT_SPEED)
{
    return rotate(90, speed);
}

bool BasicMovements::rotateRight(float speed = DEFAULT_SPEED)
{
    return rotate(-90, speed);
}

bool BasicMovements::rotateBackwards(float speed = DEFAULT_SPEED)
{
    return rotate(180, speed);
}

/*
 * Turns left around a corner. This is not the same as rotate left.
 *
 * Returns: false if obstacle was found otherwise true
**/
bool BasicMovements::turnLeft()
{
    initialiseEncoder();

    float wishLeftEncoder  = leftEncoder + (1 / (RAD_RADIUS * 2)) * (CELL_LENGTH - ROB_BASE / 2 + ROB_BASE / 2 * PI / 2);
    float wishRightEncoder = rightEncoder + (1 / (RAD_RADIUS * 2)) * (CELL_LENGTH - ROB_BASE / 2 - ROB_BASE / 2 * PI / 2);

    while (ros::ok()) {
        ros::spinOnce();

        if (fabs((wishLeftEncoder - leftEncoder)) < 0.5) {
            stop();
            return true;
        }

        move((CELL_LENGTH - ROB_BASE / 2) / 3, -PI / 2 / 3);
    }

    return false;
}

/*
 * Turns right around a corner. This is not the same as rotate right.
 *
 * Returns: false if obstacle was found otherwise true
**/
bool BasicMovements::turnRight()
{
    initialiseEncoder();

    float wishLeftEncoder  = leftEncoder + (1 / (RAD_RADIUS * 2)) * (CELL_LENGTH - ROB_BASE / 2 + ROB_BASE / 2 * -PI / 2);
    float wishRightEncoder = rightEncoder + (1 / (RAD_RADIUS * 2)) * (CELL_LENGTH - ROB_BASE / 2 - ROB_BASE / 2 * -PI / 2);

    while (ros::ok()) {
        ros::spinOnce();
        ROS_INFO("rightEncoder %f, wishRightEncoder %f", rightEncoder, wishRightEncoder);

        if (fabs((wishRightEncoder - rightEncoder)) < 0.5) {
            stop();
            return true;
        }

        move((CELL_LENGTH - ROB_BASE / 2) / 3, PI / 2 / 3);
    }

    return false;
}

/********************** HELPERS *****************************/

void BasicMovements::encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
    if (CALLBACK_DEBUG) {
        ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
    }

    encoderInitialized = true;
    leftEncoder  = msg->encoderLeft;
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
