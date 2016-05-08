#include "ros/ros.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/SensorPacket.h"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <fstream>

#define LOG
#define DEBUG

using namespace std;

ros::ServiceClient reset_enc;
create_fundamentals::ResetEncoders srv_renc;

/** Protoypes */
void reset_encoders(void);
        
/* logging the received wheel encoder values (debugging) */
ofstream datafile;

/* hold the last received values for the wheel encoders
 * initialy and after running `reset_encoders()', the value is zero */
float left_encoder;
float right_encoder;

#define DPRINT(msg) \
  (ROS_INFO("[%f,%f] %s", left_encoder, right_encoder, (msg).c_str()))

/* this is the value in radians our robot needs to actually go one meter
 * all other radian values or operations where calculated relative to
 * this value */
#define ONE_METER_IN_RAD 30.798
#define TWE_CM_IN_RAD (ONE_METER_IN_RAD / 5)
#define NTY_DEGREES_IN_RAD (ONE_METER_IN_RAD*0.196349)

//UNUSED YET
#define PI 3.1415
#define ROBO_R 0.036
#define ROBO_B 0.258

#define hz 25 // 25 times per second, use as loop rate

/**
 * go - build and send an diff drive service request.
 *      This produces a straight motion of `radians_per_second'.
 *      Call with radians_per_second = 0 to stop the robot.
 */
void go(create_fundamentals::DiffDrive srv,
        ros::ServiceClient diff_drive,
        float radians_per_second)
{
        ROS_INFO("setting speed: %f", radians_per_second);
        srv.request.left = srv.request.right = radians_per_second;
        diff_drive.call(srv);
}

/**
 * rotate - same as `go' with inverted sign for left and right wheel.
 */
void rotate(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diff_drive,
            float radians_per_second)
{
        srv.request.left  =  radians_per_second;
        srv.request.right = -radians_per_second;
        diff_drive.call(srv);
}

void go_one_meter(create_fundamentals::DiffDrive srv,
                  ros::ServiceClient diff_drive)
{
        stringstream ss;
        float speed = TWE_CM_IN_RAD; // starting with  20cm per second
        float distance_driven = 0;
        float distance_to_go = ONE_METER_IN_RAD;
        float threshold = ONE_METER_IN_RAD - 5/36; //FIXME: delta in rad

        ros::Rate loop_rate(hz);

        DPRINT("initial call to go function");
        go(srv, diff_drive, speed);
        
        while(ros::ok()) {

                if(left_encoder >= threshold) {
                        DPRINT("stop robot");
                        go(srv, diff_drive, 0);
                        break;
                }

                if(left_encoder >= (distance_driven + distance_to_go*2/3)) {
                        go(srv, diff_drive, (speed*2/3));
                        distance_to_go = ONE_METER_IN_RAD - left_encoder;
                        distance_driven = left_encoder;
                        speed = (speed*2/3);
                }

                DPRINT("call spinOnce");
                ros::spinOnce();
                loop_rate.sleep();
        }

        reset_encoders();
}

void rotate_90_degrees(create_fundamentals::DiffDrive srv,
                       ros::ServiceClient diff_drive)
{
    float threshold = NTY_DEGREES_IN_RAD - 5/36;
    int stopped = 0; 

    reset_encoders();
    ros::Rate loop_rate(hz);

    rotate(srv, diff_drive, 2.5);

    while(ros::ok()){

        if (stopped > 1) break;

        if (right_encoder >= threshold)
        {
            srv.request.right = 0;
            diff_drive.call(srv);
            stopped++;
        }

        if (left_encoder >= threshold)
        {
            srv.request.left = 0;
            diff_drive.call(srv);
            stopped++;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
}

// call the reset encoders service to reset the robots encoders
// and set the global variables to zero.
void reset_encoders(void)
{
        DPRINT("reset encoders");
        reset_enc.call(srv_renc);
        left_encoder = right_encoder = 0;
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
        // update the last measured encoder values 
        left_encoder  = msg->encoderLeft;
        right_encoder = msg->encoderRight;

        // debug output
        ROS_INFO("%f, %f", msg->encoderLeft, msg->encoderRight);

#ifdef LOG
        datafile << msg->encoderLeft << "," << msg->encoderRight << "\n";
#endif
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

        reset_enc =
                n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");

        datafile.open("data.csv", ios::app);
                
#ifndef DEBUG

        /* do the square dance */
        for(int i=0;i<4; i++) {
                go_one_meter(srv, diff_drive);
                reset_encoders();
                rotate_90(srv, diff_drive);
                reset_encoders();
        }

// debugging         
#else
        reset_encoders();
        go_one_meter(srv, diff_drive);
#endif
        
        // clean up
        datafile.close();
        return 0;
}
        
        
