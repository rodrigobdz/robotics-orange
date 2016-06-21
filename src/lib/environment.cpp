#ifndef ENVIRONMENT_LIB
#define ENVIRONMENT_LIB

#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include <basic_movements.cpp>
#include <wall_recognition.cpp>
#include <wall.cpp>

class Env
{
  public:
    Env()
    {
        // Initialize ranges
        ranges = *(new std::vector<float>(LASER_COUNT));
        // Subscribe to filtered laser scan
        laser = nh.subscribe("scan_filtered", 1, &Env::laserCallback, this);
    }

    bool align(void);

  private:
    ros::NodeHandle nh;
    ros::Subscriber laser;
    std::vector<float> ranges;
    std::vector<Wall*> walls;
    bool DEBUG = true;

    // External libraries
    BasicMovements basicMovements;
    WallRecognition wall_recognition;

    // Private functions
    bool alignToSingleWall(void);
    Wall* getClosestWallInFront(void);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

bool Env::align(void)
{
    int countWalls = 0;
    // align to first wall
    while (ros::ok()) {
        walls = wall_recognition.getWalls();
        countWalls = walls.size();

        if (DEBUG) {
            for (int i = 0; i < countWalls; i++) {
                float a = walls[i]->getAngleInDegrees();
                ROS_INFO("Angle %i %f",i , a);
            }
        }

        if (countWalls > 0) {
            alignToSingleWall();
            break;
        }

        // Drive until wall in sight
        basicMovements.drive(0.5);
    }

    if (wall_recognition.getLeftWall(walls) != NULL) {
        basicMovements.rotateLeft();
    } else {
        basicMovements.rotateRight();
    }

    // align to second wall
    while (ros::ok()) {
        if (wall_recognition.getFrontWall(walls) != NULL) {
            alignToSingleWall();
            break;
        }
        basicMovements.driveWall(0.5);
    }

    return true;
}


/************************************************************/
/*                         Helpers                          */
/************************************************************/

// get robot in correct angle and distance to one wall
bool Env::alignToSingleWall(void)
{
    ros::Rate r(LOOP_RATE);
    Wall* wall;
    float angleErrorMarginInRadians = 0.2;
    while (ros::ok()) {
        // Check the angle again and leave if its ok
        // otherwise enter another loop
        wall = getClosestWallInFront();

        if (wall) {
            if(DEBUG) {
                ROS_INFO("Wall angle %f ", wall->getAngleInDegrees());
            }

            // If angle of wall in front and distance under margin error
            // then error acceptable.
            bool angleIsAcceptable = fabs(wall->getAngleInRadians()) < angleErrorMarginInRadians;
            bool distanceIsAcceptable = wall->getDistanceInMeters() - CELL_CENTER < 0.1;

            if (DEBUG) {
                ROS_INFO("Wall distance = %f ",wall->getDistanceInMeters());
                ROS_INFO("Drive distance %f",wall->getDistanceInMeters() - CELL_CENTER);
            }

            if (angleIsAcceptable && distanceIsAcceptable) {
                break;
            }

            basicMovements.rotate(wall->getAngleInDegrees());
            basicMovements.drive(wall->getDistanceInMeters() - CELL_CENTER);
        }
    }
    return true;
}

/* Call wall_recognition.getWalls for a fresh set of collected walls.
   Return the wall which most direct in front of the robot. */
Wall* Env::getClosestWallInFront(void)
{
    walls = wall_recognition.getWalls();
    int countWalls = walls.size();
    if (countWalls == 0) {
        return NULL;
    }

    Wall* wallInFront = walls[0];
    for (int i = 0; i < countWalls; i++) {
        if (fabs(walls[i]->getAngleInRadians()) < fabs(wallInFront->getAngleInRadians())) {
            wallInFront = walls[i];
        }
    }

    return wallInFront;
}

void Env::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
}

#endif
