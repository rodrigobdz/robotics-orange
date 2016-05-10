#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

#define MAXIMUM_SPEED 10
#define ONE_METER_IN_RAD 30.798
// Distances are given in meters
#define SAFETY_DIS 0.15 // Minimum distance to keep when driving
#define CELL_LENGTH 0.80 
#define DEBUG false // Constant to define if output should be printed

void stop() 
{
  ros::init(argc, argv, "stop");
  ros::NodeHandle n;
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  create_fundamentals::DiffDrive srv;

  if(DEBUG) ROS_INFO("diffDrive 0 0");

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