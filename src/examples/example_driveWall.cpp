#include <signal.h>
#include <basic_movements.cpp>
#include <wall_recognition.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");
    signal(SIGINT, stopMotors);
    ROS_INFO("Test");

    BasicMovements basicMovements;
	bool success = basicMovements.driveWall(CELL_LENGTH);
    basicMovements.stop();

    ROS_INFO("Test");
    ROS_INFO("Success %s", success ? "true" : "false");

    return 0;
}
