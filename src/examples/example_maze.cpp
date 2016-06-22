#include <signal.h>
#include <basic_movements.cpp>
#include <maze.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_maze");
    signal(SIGINT, stopMotors);

    Maze maze;
    std::vector<Position> possiblePositions = maze.initializePositions();
    std::vector<int> wallsRobot{0, 3, 2};

    ROS_INFO("possiblePositions for deletion = %lu", possiblePositions.size());
    possiblePositions = maze.findPossiblePositions(possiblePositions, wallsRobot);
    ROS_INFO("possiblePositions after deletion = %lu", possiblePositions.size());

    for (int i = 0; i < possiblePositions.size(); i++) {
        possiblePositions[i].print();
    }

    ROS_INFO("UpdatePositions");
    possiblePositions= maze.updatePositions(possiblePositions, true, 1);

    for (int i = 0; i < possiblePositions.size(); i++) {
        possiblePositions[i].print();
    }

    return 0;
}
