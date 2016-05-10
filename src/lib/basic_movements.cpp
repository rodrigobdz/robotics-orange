// Needed includes for this library to work
// #include "ros/ros.h"
// #include <cstdlib>
// #include "create_fundamentals/DiffDrive.h"
// #include "create_fundamentals/SensorPacket.h"
// #include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/ResetEncoders.h"
#include "callbacks.cpp"

#define MAXIMUM_SPEED 10
#define ONE_METER_IN_RAD 30.798
// Distances are given in meters
#define SAFETY_DIS 0.15 // Minimum distance to keep when driving
#define CELL_LENGTH 0.80 
#define DEBUG false // Defines if output should be printed
#define LOOP_RATE 16 // Used for loop rate

namespace basic_movements {
  float leftEncoder;
  float rightEncoder;

  void stop(create_fundamentals::DiffDrive srv, ros::ServiceClient diffDrive) 
  {
    if(DEBUG) {
      ROS_INFO("diffDrive 0 0");
    }

    srv.request.left  = 0;
    srv.request.right = 0;
    diffDrive.call(srv);
  }

  /*
   * Params: distance in meter
   * Returns: false if obstacle was found otherwise true
  **/
  bool driveForward(ros::NodeHandle n, ros::ServiceClient diffDrive, create_fundamentals::DiffDrive srv, float distance) 
  {

    // Subscribe to laser scanner
    ros::NodeHandle laser;
    ros::Subscriber subLaser = laser.subscribe("scan_filtered", 1, callbacks::laserCallback);
    
    // Subscribe to reset encoders
    ros::ServiceClient resetEncodersClient;
    create_fundamentals::ResetEncoders resetEncodersService;
    resetEncodersClient = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

    // Subscribe to encoder
    ros::Subscriber subEncoder = n.subscribe("sensor_packet", 1, callbacks::encoderCallback);

    // TODO: Check if obstacle is in front and return if yes

    float threshold  = ONE_METER_IN_RAD - (ONE_METER_IN_RAD*0.025);
    float speedLeft  = 3;
    float speedRight = 3;

    helpers::resetEncoders(resetEncodersClient, resetEncodersService);
    ros::Rate loop_rate(LOOP_RATE);
    
    while(ros::ok()) {
      // TODO: Check if obstacle is in front
      if ((speedRight == 0) && (speedLeft == 0)) break;

      if (rightEncoder >= threshold) {
        speedRight = 0;
      }

      if (leftEncoder >= threshold) {
        speedLeft = 0;
      }

      srv.request.left  = speedLeft;
      srv.request.right = speedRight;
      diffDrive.call(srv);

      ros::spinOnce();
      loop_rate.sleep();
    }

    helpers::resetEncoders(resetEncodersClient, resetEncodersService);

    return true;
  }

  /*
   * Params: distance in meter
   * Returns false if obstacle was found otherwise true
  **/
  bool driveBackwards(float distance) 
  {
    return true;
  }

  void rotateLeft(float angle) 
  {

  }

  void rotateRight(float angle)
  {
    
  }
}