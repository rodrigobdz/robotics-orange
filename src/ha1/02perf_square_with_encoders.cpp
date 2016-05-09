#include "ros/ros.h"
#include "create_fundamentals/ResetEncoders.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"

#define ONE_METER_IN_RAD 30.798
#define TWE_CM_IN_RAD (ONE_METER_IN_RAD / 5)
#define NTY_DEGREES_IN_RAD (ONE_METER_IN_RAD*0.196349)
#define hz 16 // 16 times per second, use as loop rate

ros::ServiceClient reset_enc;
create_fundamentals::ResetEncoders srv_renc;

/** Protoypes */
void reset_encoders(void);

float left_encoder;
float right_encoder;

void go(create_fundamentals::DiffDrive srv,
        ros::ServiceClient diff_drive,
        float radians_per_second)
{
        srv.request.left = srv.request.right = radians_per_second;
        diff_drive.call(srv);
}

void rotate(create_fundamentals::DiffDrive srv,
            ros::ServiceClient diff_drive,
            float radians_per_second, int direction)
{
        srv.request.left  =  (-radians_per_second)*direction;
        srv.request.right = radians_per_second*direction;
        diff_drive.call(srv);
}

void reset_encoders(void)
{
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
}

void rotate_90_degrees(create_fundamentals::DiffDrive srv,
                       ros::ServiceClient diff_drive, int direction)
{
    reset_encoders();

    float threshold = NTY_DEGREES_IN_RAD - NTY_DEGREES_IN_RAD*0.04; 

    float speed_left_r = (-2.5)*direction;
    float speed_right_r = 2.5*direction;

    ros::Rate loop_rate(hz);

    rotate(srv, diff_drive, 2.5, direction);

    ros::spinOnce();

    while(ros::ok()){

        if ((speed_right_r == 0) && (speed_left_r == 0)) break;

        if (((right_encoder >= threshold) && (right_encoder > 0)) || 
              (((right_encoder*(-1)) >= threshold) && (right_encoder < 0)))
        { 
          speed_right_r = 0;
        }

        if (((left_encoder >= threshold) && (left_encoder > 0)) || 
              (((left_encoder*(-1)) >= threshold) && (left_encoder < 0)))
        {
          speed_left_r = 0;
        }


        srv.request.left  = speed_left_r;
        srv.request.right = speed_right_r;
        diff_drive.call(srv);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void go_one_meter(create_fundamentals::DiffDrive srv,
                  ros::ServiceClient diff_drive)
{
        float threshold = ONE_METER_IN_RAD - (ONE_METER_IN_RAD*0.025);
        float speed_left = 3;
        float speed_right = 3;

        reset_encoders();
        ros::Rate loop_rate(hz);
        
        while(ros::ok()) {

          if ((speed_right == 0) && (speed_left == 0)) break;

          if (right_encoder >= threshold)
          {
            speed_right = 0;
          }

          if (left_encoder >= threshold)
          {
            speed_left = 0;
          }

          srv.request.left  = speed_left;
          srv.request.right = speed_right;
          diff_drive.call(srv);

          ros::spinOnce();
          loop_rate.sleep();
        }

        reset_encoders();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ros::ServiceClient diff_drive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  reset_enc = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
  
  create_fundamentals::DiffDrive srv;

 for (int i = 0; i < 4; ++i)
 {
  go_one_meter(srv, diff_drive);

  ros::Duration(0.5).sleep();

  rotate_90_degrees(srv, diff_drive, 1);

  ros::Duration(0.5).sleep();
 }

  return 0;
}