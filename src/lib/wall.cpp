#ifndef WALL_LIB
#define WALL_LIB

#include <stdlib.h>
#include <constants.cpp>

class Wall
{
  public:
    Wall(float x1, float y1, float x2, float y2)
    {
        distance = calcDistanceInMeters(x1, y1, x2, y2);
        angleInRadians = calcAngleInRadians(x1, y1, x2, y2);
    }
    Wall(float distance, float angleInRadians) : distance(distance), angleInRadians(angleInRadians) {}

    float getDistanceInMeters() const { return distance; }
    float getAngleInRadians() const { return angleInRadians; }
    float getAngleInDegrees() const { return angleInRadians*180/PI; }

    bool isLeftWall();
    bool isRightWall();
    bool isFrontWall();

  private:
    float distance;
    float angleInRadians;

    float calcDistanceInMeters(float x1, float y1, float x2, float y2);
    float calcAngleInRadians(float x1, float y1, float x2, float y2);
};

/////////////////////////////////////////////////////
////////////////////// Helpers //////////////////////
/////////////////////////////////////////////////////

/*
 * x1, y1: First point
 * x2, y2: Second point
 * px, py: Point
 *
 * Returns: Calculated distance to point in
 *          cartesian coordinate system
 */
float Wall::calcDistanceInMeters(float x1, float y1, float x2, float y2)
{
    // Variables for Hesse normal form computation
    // from two-point form
    //    a*px + b*py - c = 0
    // --------------------------
    //      √( a² + b² )
    float a = y1 - y2;
    float b = x2 - x1;
    float c = x2 * y1 - x1 * y2;
    float normalVector = sqrt(pow(a, 2) + pow(b, 2));

    return fabs((a * 0 + b * (-DISTANCE_LASER_TO_ROBOT_CENTER) - c) / normalVector);
}

/* *
 *
 * Calculate angle from robot to wall.
 *
 * Returns: Angle to wall in radians
 * */
float Wall::calcAngleInRadians(float x1, float y1, float x2, float y2)
{
    // Line 1
    float m1 = (y2 - y1) / (x2 - x1);
    float n1 = y1 - m1 * x1;

    if (isinf(m1)) {
        if (x1 > 0) {
            m1 = -100000000;
            n1 = 100000000;
        } else {
            m1 = 100000000;
            n1 = 100000000;
        }
    }

    // Line 2
    float m2 = -1 / m1;
    float n2 = 0;

    // Intersection
    float x = (n2 - n1) / (m1 - m2);
    float y = m1 * x + n1;

    float oldAngleInRadians = atan2(y, x);

    // ROS_INFO("x = %f, y = %f, angle = %f", x, y, newAngleInRadians);

    float newAngleInRadians;
    if (oldAngleInRadians > -PI / 2) {
        newAngleInRadians = oldAngleInRadians - PI / 2;
    } else {
        newAngleInRadians = oldAngleInRadians + PI * 3 / 2;
    }

    return newAngleInRadians;
}

bool Wall::isLeftWall() { return (angleInRadians > PI / 4 && angleInRadians < 3 * PI / 4); }// angle between 45 and 135 degrees

bool Wall::isRightWall() { return (angleInRadians > -3 * PI / 4 && angleInRadians < -PI / 4); } // angle between -45 and -135 degrees

bool Wall::isFrontWall() { return (angleInRadians < PI / 4 && angleInRadians > -PI / 4); } // angle between -45 and 45 degrees

#endif
