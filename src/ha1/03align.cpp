#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "../lib/ransac.cpp"
#include "../lib/basic_movements.cpp"

#define RATE 5
#define DISTANCE_LASER_TO_ROBOT_CENTER 0.125
#define CELL_CENTER 0.40
#define ERR_OFFSET 1.131

bool aligned = false;

/*void wallAlign(float wishAngle)
{
    std::vector<float> wall;
    float distance;
    float angle;

    while (true) {
        // Update sensor data
        ros::spinOnce();

        wall = ransac();
        // Check if wall was found with ransac
        if (!isnan(wall[0])) {
            // Wall was found
            float currentAngle = calculateAngle(wall);
            if (fabs(wishAngle - currentAngle) < PI / 32) {
                srv.request.left = 0;
                srv.request.right = 0;
                diffDrive.call(srv);
                return;
            } else if (currentAngle < wishAngle) {
                srv.request.left = 1;
                srv.request.right = -1;
                diffDrive.call(srv);
            } else {
                srv.request.left = -1;
                srv.request.right = 1;
                diffDrive.call(srv);
            }
        } else {
            // If no wall was found no wall to align to
            return;
        }
    }
}*/

int main(int argc, char** argv)
{
    // Set up ROS.
    ros::init(argc, argv, "align");
    ros::NodeHandle n;

    BasicMovements move;
    Ransac ransac;
    std::vector<Wall*> walls;

    // Tell ROS how fast to run this node.
    ros::Rate r(RATE);

    // Main loop.
    while (n.ok()) {

        walls = ransac.getWalls();
        ROS_INFO("Found %lu walls", (long unsigned int) walls.size());
        for(int wall = 0; wall < walls.size(); wall++){
            ROS_INFO("Distance = %f", walls[wall]->getDistance(0, -DISTANCE_LASER_TO_ROBOT_CENTER));
        }
        /*// Check if wall was recognized
        if (isnan(wall[0])) {
            aligned = false;
            // If no wall was recognized then go straight ahead
            ::srv.request.left = 2.5;
            ::srv.request.right = 2.5;
            ::diffDrive.call(::srv);
            continue; } // if (walls.size() > 1) {
        //   aligned = false;
        //   // If no wall was recognized then go straight ahead
        //   ::srv.request.left = 2.5;
        //   ::srv.request.right = 2.5;
        //   ::diffDrive.call(::srv);
        //   continue;
        //}

        // Wall was found!
        if (!aligned) {
            float distance;
            distance = distanceFromLineToPoint(wall[0], wall[1], wall[2], wall[3], 0, -DISTANCE_LASER_TO_ROBOT_CENTER);
            // angle = calculateAngle(wall);

            // If wall was found align to be at a certain angle to it
            wallAlign(PI / 2);
            aligned = true;

            // Check if robot is positioned in center of cell
            if (distance != CELL_CENTER) {
                // Drive to center in cell
                driveToDistance(CELL_CENTER);
            }
            rotateRight90();
        }*/

        r.sleep();
    }
    return 0;
  }

}
