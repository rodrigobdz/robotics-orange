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
    Wall* getWallClosestTo90(void);

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

/* Call ransac.getWalls for a fresh set of collected walls.
   Return the wall which angle is closest to ninety degrees. */
Wall* Env::getWallClosestTo90(void)
{
    walls = getWalls();
    if (walls.size() == 0) {
        return NULL;
    }

    Wall* w = walls[0];
    for (std::vector<Wall*>::iterator it = walls.begin(); it != walls.end(); ++it) {
        if (fabs((*it)->getAngle() - 90) < fabs(w->getAngle() - 90)) {
            w = *it;
        }
    }
    return w;
}

bool Env::align(void)
{
    ros::Rate r(LOOP_RATE);
    int aligned = 0;

    while (ros::ok()) {
        walls = getWalls();
        int s = walls.size();
        ROS_INFO("Size %i", s);
        if (walls.size() >= 2) {
            ROS_INFO("align call alignToSingleWall 1.");
            alignToSingleWall();
            ROS_INFO("align rotate right.");
            _basicMovements.rotateAbs(0, 2);
            ROS_INFO("align call alignToSingleWall 2.");
            alignToSingleWall();
            aligned = 2;
        }

        if (aligned == 2) {
            break;
        }
        else {
            r.sleep();
        }
    }
    return true;
}

bool Env::alignToSingleWall(void)
{
    ros::Rate r(LOOP_RATE);
    while (ros::ok()) {
        Wall* wall = getWallClosestTo90();
        if (wall) {
            _basicMovements.rotateAbs(wall->getAngle(), 1);
            r.sleep();
            _basicMovements.drive(wall->getDistance() - CELL_CENTER, 1);

            // check the angel again and leave if its ok
            // otherwise enter another loop
            wall = getWallClosestTo90();
            if (fabs(wall->getAngle() - PI/2) < 5) {
                break;
            }
        }

        r.sleep();
    }
    return true;
}

void Env::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{ 
    _ranges = msg->ranges; 
}

#endif
