#include <signal.h>
#include <basic_movements.cpp>
#include <path_finder.cpp>

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

    std::vector<int> walls = maze.scanCurrentCellInitial();
    for (int i = 0; i < walls.size(); ++i)
    {
        printf("walls[%i] = %i\n", i, walls[i]);
    }
    return 0;
}
