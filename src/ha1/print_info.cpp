#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser Scanner Parameters:");
  ROS_INFO("Ranges Size: %u", msg->ranges.size());
  ROS_INFO("Minimal measurable range: %f", msg->range_min);
  ROS_INFO("Maximal measurable range: %f", msg->range_max);
  ROS_INFO("Angle Min %f", msg->angle_min);
  ROS_INFO("Angle Max %f", msg->angle_max);
  ROS_INFO("Angle Increment %f", msg->angle_increment);
  ROS_INFO("Time Increment %f", msg->time_increment);
  ros::Duration(5.0).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan_filtered", 1, laserCallback);
  ros::spin();
  return 0;
}