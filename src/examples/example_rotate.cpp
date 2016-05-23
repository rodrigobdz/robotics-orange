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
	// Rotate 90 degrees clockwise with a velocity of 7 rad/s

	basicMovements.rotate(-90);
    walls = ransac.getWalls();
    for (int wall = 0; wall < walls.size(); wall++) {
        ROS_INFO("wall[i].angle = %f", walls[wall]->getAngle());
    }

	basicMovements.rotate(90);
    walls = ransac.getWalls();
    for (int wall = 0; wall < walls.size(); wall++) {
        ROS_INFO("wall[i].angle = %f", walls[wall]->getAngle());
    }

	basicMovements.rotate(-90);
    walls = ransac.getWalls();
    for (int wall = 0; wall < walls.size(); wall++) {
        ROS_INFO("wall[i].angle = %f", walls[wall]->getAngle());
    }

	basicMovements.rotate(90);
    walls = ransac.getWalls();
    for (int wall = 0; wall < walls.size(); wall++) {
        ROS_INFO("wall[i].angle = %f", walls[wall]->getAngle());
    }
}


