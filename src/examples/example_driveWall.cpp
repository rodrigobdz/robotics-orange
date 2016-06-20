#include <basic_movements.cpp>
#include <signal.h>
#include <constants.cpp>
#include <wall_recognition.cpp>
#include <wall.cpp>


void mySigintHandler(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");

    BasicMovements basicMovements;
    WallRecognition wall_recognition;

	signal(SIGINT, mySigintHandler);

	basicMovements.driveWall(4);

	return 0;
}
