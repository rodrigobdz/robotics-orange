#include <basic_movements.cpp>
#include <signal.h>
#include <constants.cpp>
#include <ransac.cpp>
#include <wall.cpp>


void mySigintHandler(int signal) {
    BasicMovements basicMovements;
    while(true){
        basicMovements.stop();
        ros::shutdown();
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");

    BasicMovements basicMovements;
    Ransac ransac;

	signal(SIGINT, mySigintHandler);

	basicMovements.driveWall(4);

}
