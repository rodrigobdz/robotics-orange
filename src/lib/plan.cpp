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
                basicMovements.rotate(90);
                break;
            case LEFT:
                basicMovements.rotate(180);
                break;
            case DOWN:
                basicMovements.rotate(-90);
                break;
            default:
                break;
            }
            break;
        case UP:
            switch (*it) {
            case RIGHT:
                basicMovements.rotate(-90);
                break;
            case LEFT:
                basicMovements.rotate(90);
                break;
            case DOWN:
                basicMovements.rotate(180);
                break;
            default:
                break;
            }
            break;
        case LEFT:
            switch (*it) {
            case RIGHT:
                basicMovements.rotate(180);
                break;
            case UP:
                basicMovements.rotate(-90);
                break;
            case DOWN:
                basicMovements.rotate(90);
                break;
            default:
                break;
            }
            break;
        case DOWN:
            switch (*it) {
            case RIGHT:
                basicMovements.rotate(90);
                break;
            case UP:
                basicMovements.rotate(180);
                break;
            case LEFT:
                basicMovements.rotate(-90);
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