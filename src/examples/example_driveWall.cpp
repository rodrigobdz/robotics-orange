#include <basic_movements.cpp>
#include <signal.h>
#include <constants.cpp>


void stopMotors(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");

    BasicMovements basicMovements;

	signal(SIGINT, stopMotors);

    basicMovements.driveWall(4);
}
