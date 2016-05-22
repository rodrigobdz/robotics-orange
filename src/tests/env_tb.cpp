#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <environment.cpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ransac_testbench");
  ros::NodeHandle n;
  ros::Rate r(5);

  Wall* wall;
  Env env;

  env.alignToGrid();
  return 0;

  while(ros::ok()) {
    wall = env.getWallClosestTo90();

    if(wall) {
      ROS_INFO("getAngle(): %f", wall->getAngle());
      ROS_INFO("getDistance(): %f", wall->getDistance(0,-DISTANCE_LASER_TO_ROBOT_CENTER));
    }

    r.sleep();
  }
}
