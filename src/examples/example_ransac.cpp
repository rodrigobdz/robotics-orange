#include "../lib/ransac.cpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_drive");

    Ransac ransac;
    std::vector<Wall*> walls;

    walls = ransac.getWalls();
    ROS_INFO("Found %lu walls", (long unsigned int) walls.size());
    for(int wall = 0; wall < walls.size(); wall++){
        ROS_INFO("Distance = %f", walls[wall]->getDistance(0, -DISTANCE_LASER_TO_ROBOT_CENTER));
    }
}
