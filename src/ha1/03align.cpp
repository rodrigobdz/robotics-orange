#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"



float32[] ranges;


void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ranges = msg->ranges;
}

/* *
 * Recognise a wall with ransac
 * Returns two points of the wall
 * */
float32[] ransac()
{
  int iterations = 1000
  for (i = 0; i < iterations; ++i) {
    std::srand(123456789);
    firstRandom = std::rand();
    secondRandom = std::rand();

    a = ranges[firstRandom]



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

    if (recWall() == true) {

    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
