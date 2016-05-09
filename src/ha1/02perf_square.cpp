
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>

#define ERR_OFFSET 1.131

void go_one_meter(create_fundamentals::DiffDrive srv,
                  ros::ServiceClient diffDrive)
{
        srv.request.left  = 11.111 * ERR_OFFSET;
        srv.request.right = 11.111 * ERR_OFFSET;
        diffDrive.call(srv);
        
        ros::Duration(2.5).sleep();
        
        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);
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
}

void square(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diffDrive)
{
        for(int i=0;i<4;++i) {
                go_one_meter(srv, diffDrive);
                ros::Duration(0.1).sleep();
                rotate_right_90(srv, diffDrive);
        }
}
        
int main(int argc, char **argv)
{
        ros::init(argc, argv, "perfSquare");
        ros::NodeHandle n;
        ros::ServiceClient diffDrive =
                n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        create_fundamentals::DiffDrive srv;

        //go_one_meter(srv, diffDrive, true);
        //rotate_right_90(srv,diffDrive);
        for(int i=0; i<5; i++) 
                square(srv, diffDrive);

        return 0;
}
        
        
