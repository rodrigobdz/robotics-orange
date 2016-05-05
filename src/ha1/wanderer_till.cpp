#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

create_fundamentals::DiffDrive srv;
ros::ServiceClient diffDrive;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Current Range: %f", msg->ranges[msg->ranges.size()/2]);
  ROS_INFO("%d", isnan(msg->ranges[255]));
  ROS_INFO("%f", msg->ranges[255]);
  ROS_INFO("%d", msg->ranges[255] < 1.0);

  for(int i = 0; i < 512; ++i){
    if(msg->ranges[i] < 1.0){
        ::srv.request.left = -2;
        ::srv.request.right = 2;
        ::diffDrive.call(srv);
    } else {
        ::srv.request.left = 5;
        ::srv.request.right = 5;
        ::diffDrive.call(srv);
    }

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderer");

  ros::NodeHandle wanderer;
  ros::Subscriber subLaser = wanderer.subscribe("scan_filtered", 1, laserCallback);

  diffDrive = wanderer.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ros::spin();
  return 0;
}
