#include "ros/ros.h"
#include <maze.cpp>
#include <cell.cpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_maze");

  // int32[] walls1_1 = [ 0, 1, 2, 3 ];
  // ROS_INFO("walls1_1[0] = %i", walls1_1[0];
  // Cell[] cells1 = new Cell[1];
  // cells1[1] = walls1_1;

  // vector<Row> rows = new vector<Row>;



  Maze maze;

  maze.parseMap();

  Cell[][] map = maze.getMap();
  ROS_INFO("Map[0][0] = true = %d", map[0][0].isRight());

  // maze.parseMap();

  return 0;
}