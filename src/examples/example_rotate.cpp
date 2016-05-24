#include <basic_movements.cpp>
#include <ransac.cpp>
#include <signal.h>


void stopMotors(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_rotate");

    BasicMovements basicMovements;
	Ransac ransac;

	signal(SIGINT, stopMotors);

    std::vector<Wall*> walls;
    ros::Rate loop_rate(1/3);
	// Rotate 90 degrees clockwise with a velocity of 7 rad/s

	// basicMovements.rotate(-90);
	// basicMovements.rotate(90);
	// basicMovements.rotate(-90);
	// basicMovements.rotate(90);
	basicMovements.move(0.8 / 3, PI/2 / 3);
    loop_rate.sleep();
	basicMovements.move(0, 0);

}


