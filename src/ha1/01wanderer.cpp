#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"

#define SPEED 10 // target speed of robot (straigt)
#define RANGE_SIZE 512 // size of laser_msg array
#define RANGE_MAX 5.6 // maximum measurable range of laserscanner 
#define SAFETY_DIS 0.15 // distance robot breaks before objects
#define SIDE_OFF 0.00  // on the sides, this constant is substracted

/*Note: Withe these settings, the robot will get stuck in narrow coloirs. 
If you want to get through tight spaces and risk bumping the wall, we found SIDE_OFF = 0.08 and SAFETY_DIS = 0.13
to be good values.*/

create_fundamentals::DiffDrive srv;
ros::ServiceClient diffDrive;

int cur_speed = 0; // in 10th parts (zehntel) of SPEED, does not apply to truning
float min = 0.0; // minimal distance measured by laser,
                // initialized with 0 so robot doesn't start moving without measurements.


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

/*
Changes direction about 9º-10º 
*/
void change_direction(){
      ::srv.request.left = -4;
      ::srv.request.right = 4;
      ::diffDrive.call(::srv);
}

/*
Filters out proximity of laserscanner to next visible object.
*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  // Set minimum to maximal possible distance
  min = RANGE_MAX;

  // Finde Minimum
  for (int i = 0; i < RANGE_SIZE; ++i)
  {
    // distance on the sides (aprox 0-30º and 150-180º) can be 10cm closer
    float off = 0;

    if (i<90 || i>422){
      off = SIDE_OFF;
    }
    
    if (msg->ranges[i]+off < min) min = msg->ranges[i]+off;
  }
}

int main(int argc, char **argv)
{
  ros::Time::init();

  // Laser publishes with frequency of 14.6hz on average
  ros::Rate loop_rate(15);

  ros::init(argc, argv, "Blatt1Aufgabe1");
  ROS_INFO("wanderer");

  ros::NodeHandle n;
  ::diffDrive = n.serviceClient<create_fundamentals::DiffDrive>("diff_drive");

  ros::NodeHandle laser;
  ros::Subscriber sub_laser = laser.subscribe("scan_filtered", 1, laserCallback);

  while(ros::ok()){

        ros::spinOnce();
        // if robot to close to obstacle, stopp if not stopped, 
        // then (continue) to change direction
        if (min < SAFETY_DIS)
        {
          stop();
          change_direction();
        } else {
          //soft speedup to avoid bumping into things
          if (cur_speed < 10) cur_speed++;
          ::srv.request.left = cur_speed*(SPEED/10);
          ::srv.request.right = cur_speed*(SPEED/10);
          ::diffDrive.call(::srv);
        }

        // wait for new laser data
        loop_rate.sleep();
  }

  stop();

  return 0;
}