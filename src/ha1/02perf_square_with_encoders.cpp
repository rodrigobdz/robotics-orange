
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
using namespace std;

ros::Subscriber sub;
ofstream datafile;

#define ONE_METER_IN_RAD 31.468
#define ERR_OFFSET 1.14

void walk(create_fundamentals::DiffDrive srv,
          ros::ServiceClient diffDrive,
          float time_to_spin)
{
        srv.request.left  = 11.1111 * ERR_OFFSET;
        srv.request.right = 11.1111 * ERR_OFFSET;
        diffDrive.call(srv);
        
        ros::Duration(time_to_spin).sleep();
        
        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);

        ros::spinOnce();
}

void rotate_right_90(create_fundamentals::DiffDrive srv,
                     ros::ServiceClient diffDrive)
{
        srv.request.left  =  5.628 * ERR_OFFSET; // 5.62869
        srv.request.right = -5.628 * ERR_OFFSET;
        diffDrive.call(srv);
        
        ros::Duration(1.0).sleep();

        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);

        ros::spinOnce();
}

void square(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diffDrive)
{
        for(int i=0;i<4;++i) {
                walk(srv, diffDrive, 2.5);
                ros::Duration(0.1).sleep();
                rotate_right_90(srv, diffDrive);
        }
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
        //ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
        datafile << msg->encoderLeft << "," << msg->encoderRight << "\n";
}
        
int main(int argc, char **argv)
{
        ros::init(argc, argv, "perfSquare");
        ros::NodeHandle n;
        ros::ServiceClient diffDrive =
                n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        create_fundamentals::DiffDrive srv;

        sub = n.subscribe("sensor_packet", 1, sensorCallback);
        datafile.open("data.csv", ios::app);

        
        walk(srv, diffDrive, 1.0);
        //rotate_right_90(srv,diffDrive);
        //square(srv, diffDrive);

        datafile.close();
        return 0;
}
        
        
