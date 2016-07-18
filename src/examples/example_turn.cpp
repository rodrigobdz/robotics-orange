#include <basic_movements.cpp>
#include <signal.h>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_turn");

    BasicMovements basic_movements;

	signal(SIGINT, stopMotors);

	// Rotate 90 degrees clockwise with a velocity of 7 rad/s
	// basic_movements.turnRight();
	basic_movements.turnLeft();
	// basic_movements.turnRight();
	basic_movements.turnLeft();
	// basic_movements.turnRight();
	basic_movements.turnLeft();
	// basic_movements.turnRight();
	basic_movements.turnLeft();
	basic_movements.stop();

}



