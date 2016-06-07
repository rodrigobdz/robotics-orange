#include <basic_movements.cpp>
#include <signal.h>
#include <constants.cpp>
#include <ransac.cpp>
#include <wall.cpp>


void stopMotors(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");

    BasicMovements basicMovements;
    Ransac ransac;

	signal(SIGINT, stopMotors);

	basicMovements.driveWall(4);

}
