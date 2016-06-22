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
  std::vector<Position> possiblePositions = maze.initializePositions();
  std::vector<int> wallsRobot{0,2,3};

  ROS_INFO("possiblePositions for deletion = %lu", possiblePositions.size());
  possiblePositions = maze.findPossiblePositions(possiblePositions, wallsRobot);
  ROS_INFO("possiblePositions after deletion = %lu", possiblePositions.size());

  for(int i = 0; i < possiblePositions.size(); i++){
      possiblePositions[i].print();
  }

  // std::vector<int> wallsRobotView = maze.scanCurrentCellInitial();
  // for(int i = 0; i < wallsRobotView.size(); i++) {
  // 	ROS_INFO("Wall %d", wallsRobotView[i]);
  // }

  return 0;
}
