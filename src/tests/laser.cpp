#ifndef LASER_TEST
#define LASER_TEST

#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

std::vector<float> ranges;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
}

void initialiseLaser()
{
    ranges[0] = -1;
    ros::spinOnce();
    while (ranges[0] == -1) {
        // Get laser data before driving to recognize obstacles beforehand
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_laser");
    ros::NodeHandle n;

    ros::Subscriber laserSubscriber;
    laserSubscriber = n.subscribe("scan_filtered", 1, &laserCallback);
    ranges = *(new std::vector<float>(512));

    initialiseLaser();

    ros::Rate loop_rate(5);
    while (n.ok()) {
        ros::spinOnce();

        ROS_INFO("Laser[256] = %f", ranges[256]);
        loop_rate.sleep();

    }
}

#endif

