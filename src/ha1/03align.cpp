#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"





void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);

  float min = msg->range_max;

  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < min) {
      min = msg->ranges[i];
    }
  }


  if (min < 0.2) {
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
  // Set up ROS.
  ros::init(argc, argv, "align");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate = 0.05;

  // Subscribe and  to nodes
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  create_fundamentals::DiffDrive srv;

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok()) {
    // Publish the message
    node_example->publishMessage(&pub_message);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
