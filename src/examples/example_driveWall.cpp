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

	//basicMovements.driveWall(4);
    std::vector<Wall*> walls;

    // Wall testwall = *(new Wall(0.0F, 1.0F, 1.0F, 0.0F));
    // ROS_INFO("Angle = %f", testwall.calcAngle(-1.0F, 0.0F, 0.0F, -1.0F));

    while(true){
        walls = ransac.getWalls();
        for (int i = 0; i < walls.size(); i++){
            ROS_INFO("Wall %i: Distance %f, Angle %f", i, walls[i]->getDistance(), walls[i]->getAngle());
        }
    }

}
