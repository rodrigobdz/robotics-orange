#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <maze.cpp>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_maze");
  ros::NodeHandle n;
  ros::Rate r(LOOP_RATE);

  int32[] walls1_1 = [ 0, 1, 2, 3 ];
  ROS_INFO("walls1_1[0] = %i", walls1_1[0];
  // Cell[] cells1 = new Cell[1];
  // cells1[1] = walls1_1;

  // vector<Row> rows = new vector<Row>;



  // Maze maze;

  // maze.parseMap();


  // maze.parseMap();
  return 0;

}
