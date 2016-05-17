#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "../lib/ransac.cpp"
#include "../lib/ranges.h"

#define SPEED 10 // target speed of robot (straigt)
#define RATE 5
#define DISTANCE_LASER_TO_ROBOT_CENTER 0.125
#define ONE_METER_IN_RAD 30.798
#define CELL_CENTER 0.40
#define ERR_OFFSET 1.131

int cur_speed = 0; // in 10th parts (zehntel) of SPEED, does not apply to truning
bool aligned  = false;

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
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
}

void rotateRight90()
{
    ::srv.request.left = 5.628 * ERR_OFFSET; // 5.62869
    ::srv.request.right = -5.628 * ERR_OFFSET;
    ::diffDrive.call(::srv);

    ros::Duration(1.0).sleep();

    ::srv.request.left = 0;
    ::srv.request.right = 0;
    ::diffDrive.call(::srv);
}

/*
 * Stops robot from moving if it is moving, but not if it's turning.
*/
void stop() {

  if (cur_speed == 0)
    return;
  cur_speed = 0;

  ::srv.request.left  = 0;
  ::srv.request.right = 0;
  ::diffDrive.call(::srv);
}

void driveToDistance(float wishDistance) {
  std::vector<float> wall;
    float currentDistance;


    while(true) {
      // Update sensor data
      ros::spinOnce();

      wall = ransac();
      float distance;
      // Check if wall was found with ransac
      if(!isnan(wall[0])) {
        // Wall was found

        distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, -DISTANCE_LASER_TO_ROBOT_CENTER);

        if(fabs(wishDistance - currentDistance) < 0.03) {
          srv.request.left  = 0;
          srv.request.right = 0;
          diffDrive.call(srv);
          return;
        } else if(currentDistance < wishDistance){
          srv.request.left  = -1;
          srv.request.right = -1;
          diffDrive.call(srv);
        } else {
          srv.request.left  = 1;
          srv.request.right = 1;
          diffDrive.call(srv);
        }
      } else {
        // If no wall was found no wall to align to
        return;
      }
    }
}

void wallAlign(float wishAngle)
{
  std::vector<float> wall;
  float distance;
  float angle;

  while(true) {
    // Update sensor data
    ros::spinOnce();

    wall = ransac();
    // Check if wall was found with ransac
    if(!isnan(wall[0])) {
      // Wall was found
      float currentAngle = calculateAngle(wall);
      if(fabs(wishAngle - currentAngle) < PI/32) {
        srv.request.left  = 0;
        srv.request.right = 0;
        diffDrive.call(srv);
        return;
      } else if(currentAngle < wishAngle){
        srv.request.left  = 1;
        srv.request.right = -1;
        diffDrive.call(srv);
      } else {
        srv.request.left  = -1;
        srv.request.right = 1;
        diffDrive.call(srv);
      }
    } else {
      // If no wall was found no wall to align to
      return;
    }
  }

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
  while (n.ok()) {
    ros::spinOnce();

    std::vector<Wall*> walls;
    walls = Ransac::ransac();
    // Check if wall was recognized
    if(isnan(wall[0])) {
      aligned = false;
      // If no wall was recognized then go straight ahead
      ::srv.request.left  = 2.5;
      ::srv.request.right = 2.5;
      ::diffDrive.call(::srv);
      continue;
    }
    //if (walls.size() > 1) {
    //   aligned = false;
    //   // If no wall was recognized then go straight ahead
    //   ::srv.request.left = 2.5;
    //   ::srv.request.right = 2.5;
    //   ::diffDrive.call(::srv);
    //   continue;
    //}

    // Wall was found!
    if(!aligned) {
      float distance;
      distance= distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, -DISTANCE_LASER_TO_ROBOT_CENTER);
      // angle = calculateAngle(wall);

      // If wall was found align to be at a certain angle to it
      wallAlign(PI/2);
      aligned = true;

      // Check if robot is positioned in center of cell
      if(distance != CELL_CENTER) {
        // Drive to center in cell
        driveToDistance(CELL_CENTER);
      }
      rotateRight90();
    }

    r.sleep();
  }

  return 0;
}

