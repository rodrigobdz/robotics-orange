#include <signal.h>
#include <basic_movements.cpp>
#include <wall_recognition.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_driveWall");
    signal(SIGINT, stopMotors);

    BasicMovements basicMovements;
    WallRecognition wall_recognition;

    while(true){
        ROS_INFO("Next loop");
        basicMovements.driveWall(1, 0.6);
        // basicMovements.rotateLeft();
        // basicMovements.rotateLeft();
    }
	// basicMovements.driveWall(0.7);

    // std::vector<Wall*> walls = wall_recognition.getWalls();
    // 	Wall* frontWall = wall_recognition.getFrontWall(walls);
    // 	ROS_INFO("Frontwall distance = %f", frontWall->getDistanceInMeters());

  return 0;
}
