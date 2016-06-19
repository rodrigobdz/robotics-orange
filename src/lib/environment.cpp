#ifndef ENVIRONMENT_LIB
#define ENVIRONMENT_LIB

#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include <basic_movements.cpp>
#include <ransac.cpp>
#include <wall.cpp>

class Env
{
  public:
    Env()
    {
        // Initialize ranges
        _ranges = *(new std::vector<float>(512));
        // Subscribe to filtered laser scan
        _laser = _nh.subscribe("scan_filtered", 1, &Env::laserCallback, this);
    }

    bool align(void);
    bool alignToSingleWall(void);
    Wall* getClosestWallInFront(void);

    std::vector<Wall*> getWalls(void) 
    { 
        return _ransac.getWalls(); 
    };

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _laser;
    std::vector<float> _ranges;
    std::vector<Wall*> walls;

    BasicMovements _basicMovements;
    Ransac _ransac;

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

bool Env::align(void)
{
    int countWalls = 0;
    // align to first wall
    while (ros::ok()) {
        walls = getWalls();
        countWalls = walls.size();

        for (int i = 0; i < countWalls; i++){
            float a = walls[i]->getAngleInDegrees();
            ROS_INFO("Size %f", a);
        }

        if (countWalls != 0) {
            alignToSingleWall();
            break;
        }
        _basicMovements.drive(0.5, 255);
    }

    if (_ransac.hasLeftWall()) {
        _basicMovements.rotate(-90, 1);    
    } else {
        _basicMovements.rotate(90, 1);
    }
    
    // align to second wall
    while (ros::ok()) {
        if (_ransac.hasFrontWall()) {
            alignToSingleWall();
            break;
        }
        _basicMovements.driveWall(0.5, 255);
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
    Wall* wall = getClosestWallInFront();
    while (ros::ok()) {
        if (wall) {
            _basicMovements.rotate(wall->getAngleInRadians(), 1);
            r.sleep();
            _basicMovements.drive(wall->getDistanceInMeters() + DISTANCE_LASER_TO_ROBOT_CENTER - CELL_CENTER, 1);
            // check the angel again and leave if its ok
            // otherwise enter another loop
            wall = getClosestWallInFront();
            if (fabs(wall->getAngleInRadians()) < 0.1) {
                break;
            }
        }

        r.sleep();
    }
    return true;
}

/* Call ransac.getWalls for a fresh set of collected walls.
   Return the wall which most direct in front of the robot. */
Wall* Env::getClosestWallInFront(void)
{
    walls = getWalls();
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
    _ranges = msg->ranges; 
}

#endif
