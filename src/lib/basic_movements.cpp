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
#define DEBUG false // Constant to define if output should be printed

namespace basic_movements {
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
  bool driveForward(float distance) 
  {
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
