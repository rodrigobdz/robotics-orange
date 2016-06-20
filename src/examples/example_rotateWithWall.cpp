#include <basic_movements.cpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_rotateWithWall");

	BasicMovements basicMovements;
	ROS_INFO("Test");
    basicMovements.rotateLeft();
    basicMovements.stop();
    basicMovements.rotateRight();
    basicMovements.stop();
    basicMovements.rotateLeft();
}
