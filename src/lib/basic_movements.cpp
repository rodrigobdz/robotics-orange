#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

// Distances are given in meters
#define MAXIMUM_SPEED 10
#define SAFETY_DIS 0.15 // Minimum distance to keep when driving
#define CELL_LENGTH 0.80 
#define ONE_METER_IN_RAD 30.798
#define DEBUG false

void stop() 
{
  
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