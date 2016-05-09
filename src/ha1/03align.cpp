#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "helpers/03ransac.cpp"
#include "helpers/constants.h"

#define SPEED 10 // target speed of robot (straigt)
#define RATE 5
#define DISTANCE_LASER_TO_ROBOT_CENTER -0.125
#define ONE_METER_IN_RAD 30.798

int cur_speed = 0; // in 10th parts (zehntel) of SPEED, does not apply to truning

float left_encoder;
float right_encoder;

create_fundamentals::DiffDrive srv;
create_fundamentals::ResetEncoders srv_renc;
ros::ServiceClient diffDrive;
ros::ServiceClient reset_enc;

void resetEncoders(void)
{
        reset_enc.call(srv_renc);
        left_encoder = right_encoder = 0;
}

void sensorCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
{
        left_encoder = msg->encoderLeft;
        right_encoder = msg->encoderRight;
  //ROS_INFO("left encoder: %f, right encoder: %f", msg->encoderLeft, msg->encoderRight);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ranges = msg->ranges;
}

/*
 * Stops robot from moving if it is moving, but not if it's turning.
*/
void stop() {

  if (cur_speed == 0) return;
  cur_speed = 0;

  ::srv.request.left = 0;
  ::srv.request.right = 0;
  ::diffDrive.call(::srv);

}

void wallAlign(float angle_in_rad)
{
        resetEncoders();
        ros::Rate loop_rate(16);
        ros::spinOnce();
        
        // rotate left if `actual_angel' is greater than 0
        // rotate right otherwise
        float actual_angle = angle_in_rad - PI/2;
        int direction = (actual_angle > 0) ? 1 : -1;
        if(actual_angle < 0) actual_angle *= -1;
        float threshold =
                actual_angle - actual_angle * ONE_METER_IN_RAD * 0.13; 
        
        float speed_left_r = 2.5 * direction;
        float speed_right_r = 2.5 * direction * -1;
        
        while(ros::ok()) {

                if((speed_right_r == 0) && (speed_left_r == 0)) break;

                if(((right_encoder >= threshold) && (right_encoder > 0)) ||
                   (((right_encoder*(-1) >= threshold)) && (right_encoder < 0)))

                        speed_right_r = 0;

                if(((left_encoder >= threshold) && (left_encoder > 0)) ||
                   (((left_encoder*(-1) >= threshold)) && (left_encoder < 0)))

                        speed_left_r = 0;
                
                srv.request.left = speed_left_r;
                srv.request.right = speed_right_r;
                diffDrive.call(srv);
                ros::spinOnce();
                loop_rate.sleep();
        }

        resetEncoders();
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "align");
  ros::NodeHandle n;

  // Subscribe and  to nodes
  ros::Subscriber sub = n.subscribe("sensor_packet", 1, sensorCallback);
  ::diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");
  ::reset_enc = n.serviceClient<create_fundamentals::ResetEncoders>("reset_encoders");
  
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
      distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, 0);
      ROS_INFO("Distance = %f", distance);
      ROS_INFO("ax = %f, ay = %f", wall[0], wall[1]);
      ROS_INFO("bx = %f, by = %f", wall[2], wall[3]);
      float m = (wall[3] - wall[1]) / (wall[2] - wall[0]);
      float n = wall[1] - m * wall[0];
      float angle = asin(distance/n);
      ROS_INFO("m: %f", m);
      // ROS_INFO("angle: %f", angle*180/3.1416); 
      // ROS_INFO("angle %f ", (wall[3]-wall[1])/(wall[2]-wall[0])*(0-wall[0]) + wall[1]);
    }


    r.sleep();
  }

  return 0;
}

