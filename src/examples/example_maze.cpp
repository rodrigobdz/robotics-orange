#include "ros/ros.h"
#include <maze.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

using namespace orange_fundamentals;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_maze");

  Maze maze;
  ROS_INFO("Test");
  ROS_INFO("Number of walls: %lu", maze.scanCurrentCell().size());

  return 0;
}
