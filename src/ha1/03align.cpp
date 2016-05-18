#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/ResetEncoders.h"
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <ransac.cpp>
#include <basic_movements.cpp>

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

    BasicMovements basicMovements;
    Ransac ransac;
    std::vector<Wall*> walls;

    // Tell ROS how fast to run this node.
    ros::Rate r(RATE);

    // Main loop.
    while (n.ok()) {
        walls = ransac.getWalls();

        // Check if wall was recognized
        if (walls.size() < 1) {
            // If no wall was recognized then go straight ahead
            basicMovements.drive(0.05);
        } else {
            // Wall was found!
            ROS_INFO("Angle = %f", walls[0]->getAngle());
            basicMovements.rotate(walls[0]->getAngle());
        }

        r.sleep();
    }
    return 0;
}
