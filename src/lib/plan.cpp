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
    bool executeSmooth(std::vector<int> plan, int direction);

    std::vector<int> duplicatePlan(std::vector<int> plan);
    std::vector<int> makeSmoothPlan(std::vector<int> plan);

  private:
    bool DEBUG = true;

    int TURNRIGHT = -1;
    int STRAIGHT = 0;
    int TURNLEFT = 1;
};

bool Plan::execute(std::vector<int> plan, int direction)
{
    BasicMovements basic_movements;
    int lastDirection = direction;
    bool executionSuccessful = true;

    if (DEBUG) {
        ROS_INFO("Execute plan");
    }

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        if (DEBUG) {
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

        if (!executionSuccessful) {
            return false;
        }
    }

    return executionSuccessful;
}

bool Plan::executeSmooth(std::vector<int> plan, int direction)
{
    if (plan.size() == 0) {
        return true;
    }
    BasicMovements basic_movements;
    int lastDirection = direction;
    bool executionSuccessful = true;

    std::vector<int> smoothPlan = makeSmoothPlan(plan);

    if (DEBUG) {
        ROS_INFO("Execute smooth plan");
    }

    if (DEBUG) {
        ROS_INFO("Drive in %i, lastDirection = %i", smoothPlan[0], lastDirection);
    }

    switch (lastDirection) {
    case RIGHT:
        switch (smoothPlan[0]) {
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
        switch (smoothPlan[0]) {
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
        switch (smoothPlan[0]) {
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
        switch (smoothPlan[0]) {
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
    executionSuccessful = executionSuccessful && basic_movements.driveWall(CELL_LENGTH / 2, 8);
    lastDirection = smoothPlan[0];

    if (!executionSuccessful) {
        return false;
    }

    for (int i = 1; i < smoothPlan.size() - 1; ++i) {
        if (DEBUG) {
            ROS_INFO("Drive %i", smoothPlan[i]);
        }

        if (smoothPlan[i] == TURNRIGHT) {
            executionSuccessful = basic_movements.turnRight();
        } else if (smoothPlan[i] == TURNLEFT) {
            executionSuccessful = basic_movements.turnLeft();
        } else if (smoothPlan[i] == STRAIGHT) {
            executionSuccessful = basic_movements.driveWall(CELL_LENGTH, 8);
        }

        basic_movements.driveWall(CELL_LENGTH / 2, 8);
        basic_movements.stop();

        if (!executionSuccessful) {
            return false;
        }
    }

    return true;
}

std::vector<int> Plan::duplicatePlan(std::vector<int> plan)
{
    std::vector<int> duplicatedPlan;
    for (int i = 0; i < plan.size(); ++i) {
        duplicatedPlan.push_back(plan[i]);
        duplicatedPlan.push_back(plan[i]);
    }
    return duplicatedPlan;
}

std::vector<int> Plan::makeSmoothPlan(std::vector<int> plan)
{
    if (plan.size() == 0) {
        return plan;
    }

    std::vector<int> duplicatedPlan = duplicatePlan(plan);
    std::vector<int> smoothPlan;

    smoothPlan.push_back(duplicatedPlan.front());
    for (int i = 1; i < duplicatedPlan.size() - 2; i = i + 2) {
        if (duplicatedPlan[i + 1] - duplicatedPlan[i] == TURNRIGHT || duplicatedPlan[i + 1] - duplicatedPlan[i] == 3) {
            smoothPlan.push_back(TURNRIGHT);
        } else if (duplicatedPlan[i + 1] - duplicatedPlan[i] == TURNLEFT ||
                   duplicatedPlan[i + 1] - duplicatedPlan[i] == -3) {
            smoothPlan.push_back(TURNLEFT);
        } else {
            smoothPlan.push_back(STRAIGHT);
        }
    }
    smoothPlan.push_back(0);

    return smoothPlan;
}

#endif // PLAN_LIB
