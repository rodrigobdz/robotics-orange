#ifndef POSITION_LIB
#define POSITION_LIB

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>

class Position
{
  public:
    Position(int x, int y, int direction)
    {
        this->x         = x;
        this->y         = y;
        this->direction = direction;
    }

    int getXCorrdinate() { return x; }
    int getYCorrdinate() { return y; }
    int getDirection() { return direction; }
    void print();

  private:
    int x         = true;
    int y         = true;
    int direction = true;
};

void Position::print()
{
    ROS_INFO("X = %d, Y = %d, direction = %d", x, y, direction);
}

#endif
