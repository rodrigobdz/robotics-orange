#include "ros/ros.h"
#include <wall_recognition.cpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_wall_recognition");
    ros::NodeHandle n;

    WallRecognition wall_recognition;
    std::vector<Wall*> walls;
    ros::Rate r(LOOP_RATE);

    while (n.ok()) {
        walls = wall_recognition.getWalls();
        ROS_INFO("Found %lu walls", (long unsigned int)walls.size());
        for (int wall = 0; wall < walls.size(); wall++) {
            ROS_INFO("Distance = %f", walls[wall]->getDistanceInMeters());
            ROS_INFO("Angle = %f", walls[wall]->getAngleInRadians());
            //ROS_INFO("wall[i].angle = %f", walls[wall]->getAngleInRadians());
        }
        r.sleep();
    }
}
