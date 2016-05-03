#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

#define SPEED 10;

create_fundamentals::DiffDrive srv;
ros::ServiceClient diffDrive;
int stopped = 0;

void soft_stop(int sb) {

  if (stopped == 1) return;
  stopped = 1;
  for (int j = 10; j >= 0; --j)
    {
      ::srv.request.left = j;
      ::srv.request.right = j;
      ::diffDrive.call(::srv);
      ROS_INFO("sleep %d", j);
      ros::Duration(0.05).sleep();
    }
}

void change_direction(){
    ::srv.request.left = -5;
    ::srv.request.right = 5;
    ::diffDrive.call(::srv);
    ros::Duration(1.5).sleep();
    ::srv.request.left = SPEED;
    ::srv.request.right = SPEED;
    ::diffDrive.call(::srv);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);

  float min = msg->range_max;

  for (int i = 0; i < msg->ranges.size(); ++i)
  {
    if (msg->ranges[i] < min)
    {
      min = msg->ranges[i];
    }
  }


  if (min < 0.2)
  {
    ROS_INFO("Avoid Crash.");
    soft_stop(stopped);
    change_direction();
  } else {
    ROS_INFO("diffDrive 10 10");
    ::srv.request.left = SPEED;
    ::srv.request.right = SPEED;
    ::diffDrive.call(::srv);
    stopped = 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ROS_INFO("square_moving");

  ros::NodeHandle n;
  ::diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ros::NodeHandle laser;
  ros::Subscriber sub_laser = laser.subscribe("scan_filtered", 1, laserCallback);

  //ros::Duration(2.0).sleep();

  ros::spin();
  return 0;
}