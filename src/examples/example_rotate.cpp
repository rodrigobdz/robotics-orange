#include "../lib/basic_movements.cpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_rotate");

	BasicMovements basicMovements;
	// Rotate 90 degrees clockwise with a velocity of 7 rad/s
	basicMovements.rotate(-90,7);
}