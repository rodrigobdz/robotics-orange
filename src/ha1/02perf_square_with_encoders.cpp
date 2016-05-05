
#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Vector3.h>

using namespace std;

ros::ServiceClient reset_encoders;
create_fundamentals::ResetEncoders srv_renc;

/* logging the received wheel encoder values (debugging) */
ofstream datafile;

/* hold the last received values for the wheel encoders
 * initialy and after running `reset_encoders()', the value is zero */
float left_encoder;
float right_encoder;

/* this is the value in radians our robot needs to actually go one meter
 * all other radian values or operations where calculated relative to
 * this value */
#define ONE_METER_IN_RAD 31.468
#define 20_CM_IN_RAD (ONE_METER_IN_RAD / 5)

/* use this number as loop rate */
#define hz 20 

/**
 * go - build and send an diff drive service request.
 *      This produces a straight motion of `radians_per_second'.
 *      Call with radians_per_second = 0 to stop the robot.
 */
void go(create_fundamentals::DiffDrive srv,
        ros::ServiceClient diff_drive,
        float radians_per_second)
{
        srv.request.left = srv.request.right = radians_per_seconds;
        diff_drive.call(srv);
}

/**
 * rotate - same as `go' with inverted sign for left and right wheel.
 */
void rotate(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diff_drive
            float radians_per_second)
{
        srv.request.left  =  radians_per-second;
        srv.request.right = -radians_per-second;
        diffDrive.call(srv);
}

void go_one_meter(create_fundamentals::DiffDrive srv,
                  ros::ServiceClient diff_drive)
{
        float speed = 20_CM_IN_RAD; // starting with  20cm per second
        float distance_to_go = ONE_METER_IN_RAD;
        float threshold = ONE_METER_IN_RAD - 0; //FIXME: delta in rad
        
        go(srv, diff_drive, speed);
        
        while(ros::ok()) {

                if(left_encoder >= threshold) {
                        go(srv, diff_drive, 0);
                        break;
                }
                
                if(left_encoder >= distance_to_go*2/3)
                        go(srv, diff_drive, speed/2);
                
                ros::spinOnce();
                loop_rate.sleep();
        }
}

void rotate_90_degrees(create_fundamentals::DiffDrive srv,
                       ros::ServiceClient diff_drive)
{
        ; // TODO:
}

// call the reset encoders service to reset the robots encoders
// and set the global variables to zero.
void reset_encoders(void)
{
        reset_encoders.call(srv_renc);
        left_encoder = right_encoder = 0;
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
        // update the last measured encoder values 
        left_encoder  = msg->encoderLeft;
        right_encoder = msg->encoderRight;

        // debug output
        ROS_INFO("%f, %f", msg->encoderLeft, msg->encoderRight);
        datafile << msg->encoderLeft << "," << msg->encoderRight << "\n";
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "perfSquareSens");
        ros::NodeHandle n;
        create_fundamentals::DiffDrive srv;
        ros::ServiceClient diff_drive =
                n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
        ros::Subscriber sub_sensor =
                n.subscribe("sensor_packet", 1, sensorCallback);

        reset_encoders =
                n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

        datafile.open("data.csv", ios::app);
        ros::Rate loop_rate(hz);

        
        /** do the square dance */

#if 0
        for(int i=0;i<4; i++) {
                go_one_meter(srv, diff_drive);
                reset_encoders();
                rotate_90(srv, diff_drive);
                reset_encoders();
        }

// debugging         
#elif
        go_one_meter(srv, diff_drive);
#endif
        
        // clean up
        datafile.close();
        return 0;
}
        
        
