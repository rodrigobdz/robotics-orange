#include <signal.h>
#include <basic_movements.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_drive");
    signal(SIGINT, stopMotors);

    BasicMovements basic_movements;
    bool success = basic_movements.drive(CELL_LENGTH*2);

    ROS_INFO("Success %s", success ? "true" : "false");

    return 0;
}
