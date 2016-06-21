#include <signal.h>
#include <basic_movements.cpp>
#include <maze.cpp>

void mySigintHandler(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
    ros::shutdown();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_maze");

  signal(SIGINT, mySigintHandler);

  Maze maze;
  std::vector<int> wallsRobotView = maze.scanCurrentCellInitial();
  ROS_INFO("Test");
  for(int i = 0; i < wallsRobotView.size(); i++) {
  	ROS_INFO("Wall %d", wallsRobotView[i]);
  }


  return 0;
}
