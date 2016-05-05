
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>

ros::Subscriber sub;

#define ERR_OFFSET 1.131
#define ROT_ERR_OFFSET 1.13

void go_one_meter(create_fundamentals::DiffDrive srv,
                  ros::ServiceClient diffDrive,
                  bool with_sensor)
{
        srv.request.left  = 11.111*ERR_OFFSET;
        srv.request.right = 11.111*ERR_OFFSET;
        diffDrive.call(srv);
        
        ros::Duration(2.5).sleep();
        
        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);

        // put sensor data to stdout
        if(with_sensor)
                ros::spinOnce();
}

void rotate_right_90(create_fundamentals::DiffDrive srv,
                     ros::ServiceClient diffDrive)
{
        srv.request.left  =  5.628 * ROT_ERR_OFFSET; // 5.62869
        srv.request.right = -5.628 * ROT_ERR_OFFSET;
        diffDrive.call(srv);
        
        ros::Duration(1.0).sleep();

        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);
}

void square(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diffDrive,
            bool with_sensor)
{
        for(int i=0;i<4;++i) {
                if(with_sensor)
                        ros::spinOnce();
                
                go_one_meter(srv, diffDrive, with_sensor);
                ros::Duration(0.1).sleep();
                rotate_right_90(srv, diffDrive);
        }
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
        ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}
        
int main(int argc, char **argv)
{
        ros::init(argc, argv, "perfSquare");
        ros::NodeHandle n;
        ros::ServiceClient diffDrive =
                n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        create_fundamentals::DiffDrive srv;

        sub = n.subscribe("sensor_packet", 1, sensorCallback);
        
        //go_one_meter(srv, diffDrive, true);
        //rotate_right_90(srv,diffDrive);
        for(int i=0; i<5; i++) 
                square(srv, diffDrive, false);

        return 0;
}
        
        
