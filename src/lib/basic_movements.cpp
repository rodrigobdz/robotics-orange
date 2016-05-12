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
      encoderSubscriber = n.subscribe("sensor_packet", 1, &BasicMovements::encoderCallback, this);
      diffDriveClient = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

      // Set up laser callback
      laserSubscriber = n.subscribe("scan_filtered", 1, &BasicMovements::laserCallback, this);

      // Set up reset encoders client
      resetEncodersClient = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
    }

    void stop();
    bool drive(float distance, int speed = MEDIUM_SPEED);
    bool rotate(float angle, int speed = MEDIUM_SPEED);

  private: 
    static const int MAXIMUM_SPEED      = 10;
    static const int MEDIUM_SPEED       = 5;
    static const int SLOW_SPEED         = 1;
    static const float ONE_METER_IN_RAD = 30.798;
    // Distances are given in meters
    static const float SAFETY_DIS       = 0.15; // Minimum distance to keep when driving
    static const float CELL_LENGTH      = 0.80; 
    static const bool DEBUG             = false; // Defines if output should be printed
    static const int LOOP_RATE          = 16; // Used for loop rate


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
 * Returns: false if obstacle was found otherwise true
**/
bool BasicMovements::drive(float distance, int speed) 
{
  return true;
}

/*
 * Params: if angle positive robot will rotate clockwise
 *         else if negative counter clockwise
**/
bool BasicMovements::rotate(float angle, int speed)
{
  return true; 
}



/********************** HELPERS *****************************/



void BasicMovements::encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  if(DEBUG) {
    ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
  }
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