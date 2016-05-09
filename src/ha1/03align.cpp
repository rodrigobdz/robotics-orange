#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "helpers/03ransac.cpp"
#include "helpers/constants.h"

#define SPEED 10 // target speed of robot (straigt)
#define RATE 5
#define DISTANCE_LASER_TO_ROBOT_CENTER -0.125

int cur_speed = 0; // in 10th parts (zehntel) of SPEED, does not apply to truning

create_fundamentals::DiffDrive srv;
ros::ServiceClient diffDrive;

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
  //ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ranges = msg->ranges;
  // for( int i = 0; i < ranges.size(); i++){
  //     ROS_INFO("ranges[%i] = %f", i, ranges[i]);
  // }
}

/*
Stops robot from moving if it is moving, but not if it's turning.
*/
void stop() {

  if (cur_speed == 0) return;
  cur_speed = 0;

  ::srv.request.left = 0;
  ::srv.request.right = 0;
  ::diffDrive.call(::srv);

}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "align");
  ros::NodeHandle n;

  // Subscribe and  to nodes
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ::diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ros::NodeHandle laser;
  ros::Subscriber sub_laser = laser.subscribe("scan_filtered", 1, laserCallback);


  // Tell ROS how fast to run this node.
  ros::Rate r(RATE);

  // Main loop.
  std::vector<float> wall;

  while (n.ok()) {

    ros::spinOnce();

    float distance;
    wall = ransac();
    // Check if wall was found with ransac
    if(!isnan(wall[0])){
      // Wall was found
      //distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, DISTANCE_LASER_TO_ROBOT_CENTER);
      distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, 0);
      ROS_INFO("Distance = %f", distance);
    }

    //float distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, 0);

    // if(isnan(distance)){
    //   if (cur_speed < 10) cur_speed++;
    //   ::srv.request.left = cur_speed*(SPEED/10);
    //   ::srv.request.right = cur_speed*(SPEED/10);
    //   ::diffDrive.call(::srv);
    // } else if(distance > 0.45){
    //   if (cur_speed < 10) cur_speed++;
    //   ::srv.request.left = cur_speed*(SPEED/10);
    //   ::srv.request.right = cur_speed*(SPEED/10);
    //   ::diffDrive.call(::srv);
    // } else if(distance < 0.55){
    //   if (cur_speed < 10) cur_speed++;
    //   ::srv.request.left = -cur_speed*(SPEED/10);
    //   ::srv.request.right = -cur_speed*(SPEED/10);
    //   ::diffDrive.call(::srv);
    // } else{
    //     stop();
    // }

    /// ROS_INFO("ax = %f, ay = %f", wall[0], wall[1]);
    /// ROS_INFO("bx = %f, by = %f", wall[2], wall[3]);

    r.sleep();
  }

  return 0;
}

