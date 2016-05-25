#include <basic_movements.cpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_rotateWithWall");

	BasicMovements basicMovements;
	// Rotate 90 degrees clockwise with a velocity of 7 rad/s
	ROS_INFO("Test");
    // basicMovements.rotateWithWall(45);
    basicMovements.rotate(90);
    basicMovements.stop();
    //basicMovements.drive(0.5);
    basicMovements.rotate(-90);
    basicMovements.stop();
    basicMovements.rotate(90);
}
