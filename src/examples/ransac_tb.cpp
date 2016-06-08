#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <ransac.cpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ransac_testbench");
  ROS_INFO("RANSAC testbench");
  Ransac ransac;

  std::vector<Wall*> walls;

  ros::Rate r(5);
  while(ros::ok()) {
    walls = ransac.getWalls();
    if(walls.size() != 0) {
      ROS_INFO("walls[0]->getAngle(): %f", walls[0]->getAngle());
      ROS_INFO("walls[0]->getDistance(): %f", walls[0]->getDistance(0,-DISTANCE_LASER_TO_ROBOT_CENTER));
    }
  //ROS_INFO("found %d walls.", walls.size());
  r.sleep();
  }
}
