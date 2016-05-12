#include <stdlib.h>

const double pi = 3.14159;

class Wall
{
  public:
    Wall(float x1, float y1, float x2, float y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}

    float getDistance(float px, float py);
    float getAngle();
    float getX1() const { return x1; }
    float getY1() const { return y1; }
    float getX2() const { return x2; }
    float getY2() const { return y2; }

  private:
    float x1;
    float y1;
    float x2;
    float y2;
};

/*
 * x1, y1: First point
 * x2, y2: Second point
 * px, py: Point
 *
 * Returns: Calculated distance to point in
 *          cartesian coordinate system
 */
float Wall::getDistance(float px, float py)
{
    // Variables for Hesse normal form computation
    // from two-point form
    //    a*px + b*py - c = 0
    // --------------------------
    //      √( a² + b² )
    float a = Wall::y1 - Wall::y2;
    float b = Wall::x2 - Wall::x1;
    float c = Wall::x2 * Wall::y1 - Wall::x1 * Wall::y2;
    float normalVector = sqrt(pow(a, 2) + pow(b, 2));

    return fabs((a * px + b * py - c) / normalVector);
}

/* *
 *
 * Calculate angle from robot to wall.
 *
 * Returns: Angle to wall
 * */
float Wall::getAngle()
{
    float m = (Wall::y2 - Wall::y1) / (Wall::x2 - Wall::x1);
    float n = Wall::y1 - m * Wall::x1;;
    float distance = Wall::getDistance(0, 0);

    float angle = asin((distance) / n);

    // Correct angle
    if (angle < 0)
        angle = 0;
    if (angle > pi)
        angle = pi;

    if (m > 0) {
        angle = fabs(asin(distance / n) - pi);
    }

    return angle;
}

