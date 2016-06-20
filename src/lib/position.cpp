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
        this->x = x;
        this->y = y;
        this->direction = direction;
    }

    int getXCorrdinate() { return x; }
    int getYCorrdinate() { return y; }
    int getDirection() { return direction; }

  private:
    bool x = true;
    bool y = true;
    bool direction = true;
};

#endif
