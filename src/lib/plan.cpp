#ifndef PLAN_LIB
#define PLAN_LIB

#include <constants.cpp>
#include <basic_movements.cpp>

class Plan
{
    public:
        Plan(){}
    
        bool execute(std::vector<int> plan);  

    private:
        bool DEBUG = true;      
};

bool Plan::execute(std::vector<int> plan) 
{
    BasicMovements basicMovements;
    int lastDirection = UP;
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
                executionSuccessful = executionSuccessful && basicMovements.rotateLeft();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basicMovements.rotateBackwards();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basicMovements.rotateRight();
                break;
            default:
                break;
            }
            break;
        case UP:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basicMovements.rotateRight();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basicMovements.rotateLeft();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basicMovements.rotateBackwards();
                break;
            default:
                break;
            }
            break;
        case LEFT:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basicMovements.rotateBackwards();
                break;
            case UP:
                executionSuccessful = executionSuccessful && basicMovements.rotateRight();
                break;
            case DOWN:
                executionSuccessful = executionSuccessful && basicMovements.rotateLeft();
                break;
            default:
                break;
            }
            break;
        case DOWN:
            switch (*it) {
            case RIGHT:
                executionSuccessful = executionSuccessful && basicMovements.rotateLeft();
                break;
            case UP:
                executionSuccessful = executionSuccessful && basicMovements.rotateBackwards();
                break;
            case LEFT:
                executionSuccessful = executionSuccessful && basicMovements.rotateRight();
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        executionSuccessful = executionSuccessful && basicMovements.driveWall(CELL_LENGTH);
        lastDirection = *it;
    }

    return executionSuccessful;
}

#endif // PLAN_LIB
