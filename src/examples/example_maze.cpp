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
  ros::Rate r(5);

  int32[] walls1_1;
  Cell[] cells1;
  Cell[] cells2;
  Cell[] cells3;
  vector<Row> rows;

  Maze maze;


  maze.parseMap();
  return 0;

}
