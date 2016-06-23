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

    return 0;
}
