#ifndef WALL_LIB
#define WALL_LIB

#include <stdlib.h>
#include <constants.cpp>

class Wall
{
  public:
    Wall(float x1, float y1, float x2, float y2)
    {
        distance = calcDistance(x1, y1, x2, y2);
        angle = calcAngle(x1, y1, x2, y2);
    }
    Wall(float distance, float angle) : distance(distance), angle(angle) {}

    float getDistance() const { return distance; }
    float getAngle() const { return angle; }

  private:
    float distance;
    float angle;

    float calcDistance(float x1, float y1, float x2, float y2);
    float calcAngle(float x1, float y1, float x2, float y2);
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
float Wall::calcDistance(float x1, float y1, float x2, float y2)
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
float Wall::calcAngle(float x1, float y1, float x2, float y2)
{
    // Line 1
    float m1 = (y2 - y1) / (x2 - x1);
    float n1 = y1 - m1 * x1;

    // Line 2
    float m2 = -m1;
    float n2 = n1;

    // Intersection
    float x = (n2 - n1) / (m1 - m2);
    float y = m1 * x + n1;

    if (x > 0) {
        return PI + atan(y / x);
    } else {
        return atan(y / x);
    }
}

#endif
