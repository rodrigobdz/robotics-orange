#include "ros/ros.h"
#include <maze.cpp>
// #include <cell.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

using namespace orange_fundamentals;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_maze");

  // int32[] walls1_1 = [ 0, 1, 2, 3 ];
  // ROS_INFO("walls1_1[0] = %i", walls1_1[0];
  // Cell[] cells1 = new Cell[1];
  // cells1[1] = walls1_1;

  Maze maze;
  ROS_INFO("Test");

  std::vector<Row> rows = maze.getMap();
  std::vector<orange_fundamentals::Cell> cells = rows[1].cells;
  ROS_INFO("Map[0][0] = true = %lu", cells.size());

  // maze.parseMap();

  return 0;
}
