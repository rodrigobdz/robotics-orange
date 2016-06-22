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
    ROS_INFO("Position x = %d, y = %d, direction = %d", maze.getPosition().getXCoordinate(), maze.getPosition().getYCoordinate(), maze.getPosition().getDirection());

    return 0;
}
