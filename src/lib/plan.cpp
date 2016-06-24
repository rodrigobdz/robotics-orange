#ifndef PLAN_LIB
#define PLAN_LIB

#include <constants.cpp>
#include <basic_movements.cpp>
#include <maze.cpp>

class Plan
{
    public:
      Plan() {}

        bool execute(std::vector<int> plan, int direction);

    private:
        bool DEBUG = true;

};

bool Plan::execute(std::vector<int> plan, int direction)
{
    BasicMovements basic_movements;
    int lastDirection        = direction;
    bool executionSuccessful = true;

    if(DEBUG) {
        ROS_INFO("Execute plan");
    }

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        if(DEBUG) {
            ROS_INFO("Drive in %i, lastDirection = %i", *it, lastDirection);
        }

        switch (lastDirection) {
        case RIGHT:
            switch (*it) {
            case UP:
                executionSuccessful = executionSuccessful && basic_movements.rotateLeft();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basic_movements.rotateBackwards();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basic_movements.rotateRight();
                break;
            default:
                break;
            }
            break;
        case UP:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basic_movements.rotateRight();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basic_movements.rotateLeft();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basic_movements.rotateBackwards();
                break;
            default:
                break;
            }
            break;
        case LEFT:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basic_movements.rotateBackwards();
                break;
            case UP:
                executionSuccessful = executionSuccessful && basic_movements.rotateRight();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basic_movements.rotateLeft();
                break;
            default:
                break;
            }
            break;
        case DOWN:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basic_movements.rotateLeft();
                break;
            case UP:
                executionSuccessful = executionSuccessful && basic_movements.rotateBackwards();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basic_movements.rotateRight();
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        executionSuccessful = executionSuccessful && basic_movements.driveWall(CELL_LENGTH);
        lastDirection = *it;
    }

    return executionSuccessful;
}

#endif // PLAN_LIB
