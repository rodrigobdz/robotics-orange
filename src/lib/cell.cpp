#ifndef CELL_LIB
#define CELL_LIB

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>

class Cell
{
  public:
    Cell(bool right, bool top, bool left, bool bottom)
    {
        this->right = right;
        this->top = top;
        this->left = left;
        this->bottom = bottom;
    }

    bool isRight() { return this->right; }
    bool isTop() { return this->top; }
    bool isLeft() { return this->left; }
    bool isBottom() { return this->bottom; }

  private:
    bool right = true;
    bool top = true;
    bool left = true;
    bool bottom = true;
};

#endif
