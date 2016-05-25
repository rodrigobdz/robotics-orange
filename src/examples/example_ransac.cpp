#include "ros/ros.h"
#include <ransac.cpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_ransac");
    ros::NodeHandle n;

    Ransac ransac;
    std::vector<Wall*> walls;
    ros::Rate r(10);

    while (n.ok()) {
        walls = ransac.getWalls();
        ROS_INFO("Found %lu walls", (long unsigned int)walls.size());
        for (int wall = 0; wall < walls.size(); wall++) {
            ROS_INFO("  Distance = %f", walls[wall]->getDistance());
            ROS_INFO("  Angle = %f", walls[wall]->getDistance());
            //ROS_INFO("wall[i].angle = %f", walls[wall]->getAngle());
        }
        r.sleep();
    }
}
