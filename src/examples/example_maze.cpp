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
  Position position{0, 0, DOWN};
  std::vector<int> walls = maze.rows[position.getYCorrdinate()].cells[position.getXCorrdinate()].walls;
  for(int i = 0; i < walls.size(); i++){
    ROS_INFO("rows[0].cells[0] wall = %d", walls[i]);
  }

  bool compare = maze.compareWalls(position, wallsRobot);
  ROS_INFO("compare position{0, 0, DOWN} = {0,2,3} is %d", compare);
  // ROS_INFO("possiblePositions for deletion = %lu", possiblePositions.size());
  // possiblePositions = maze.compareWalls(possiblePositions, wallsRobot);
  // ROS_INFO("possiblePositions after deletion = %lu", possiblePositions.size());

  // std::vector<int> wallsRobotView = maze.scanCurrentCellInitial();
  // for(int i = 0; i < wallsRobotView.size(); i++) {
  // 	ROS_INFO("Wall %d", wallsRobotView[i]);
  // }

  return 0;
}
