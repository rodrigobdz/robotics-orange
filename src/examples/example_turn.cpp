#include <basic_movements.cpp>
#include <signal.h>
#include <constants.cpp>


void stopMotors(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_turn");

    BasicMovements basicMovements;

	signal(SIGINT, stopMotors);

	// Rotate 90 degrees clockwise with a velocity of 7 rad/s
	basicMovements.turnRight();
	basicMovements.turnLeft();

}



