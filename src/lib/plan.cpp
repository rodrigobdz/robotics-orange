#ifndef PLAN_LIB
#define PLAN_LIB

#include <constants.cpp>
#include <basic_movements.cpp>

class Plan
{
    public:
        Plan()
        {
        }
    
        bool execute(std::vector<int> plan);        
};

bool Plan::execute(std::vector<int> plan) 
{
    BasicMovements basicMovements;
    int lastDirection = UP;

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        ROS_INFO("Drive in %i, lastDirection = %i", *it, lastDirection);

        switch (lastDirection) {
        case RIGHT:
            switch (*it) {
            case UP:
                basicMovements.rotateLeft();
                break;
            case LEFT:
                basicMovements.rotateBackwards();
                break;
            case DOWN:
                basicMovements.rotateRight();
                break;
            default:
                break;
            }
            break;
        case UP:
            switch (*it) {
            case RIGHT:
                basicMovements.rotateRight();
                break;
            case LEFT:
                basicMovements.rotateLeft();
                break;
            case DOWN:
                basicMovements.rotateBackwards();
                break;
            default:
                break;
            }
            break;
        case LEFT:
            switch (*it) {
            case RIGHT:
                basicMovements.rotateBackwards();
                break;
            case UP:
                basicMovements.rotateRight();
                break;
            case DOWN:
                basicMovements.rotateLeft();
                break;
            default:
                break;
            }
            break;
        case DOWN:
            switch (*it) {
            case RIGHT:
                basicMovements.rotateLeft();
                break;
            case UP:
                basicMovements.rotateBackwards();
                break;
            case LEFT:
                basicMovements.rotateRight();
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        basicMovements.driveWall(CELL_LENGTH);
        lastDirection = *it;
    }

    return true;
}

#endif // PLAN_LIB
