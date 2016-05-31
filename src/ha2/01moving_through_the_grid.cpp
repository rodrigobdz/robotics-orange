#include "ros/ros.h"
#include "orange_fundamentals/ExecutePlan.h"
#include <basic_movements.cpp>
#include <environment.cpp>

enum directions { RIGHT = 0, UP, LEFT, DOWN };


bool execute_plan_callback(orange_fundamentals::ExecutePlan::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    BasicMovements basicMovements;
    std::vector<int> plan = req.plan;
    int lastDirection = UP;
    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        ROS_INFO("execute_plan_callback: %d", *it);

        ROS_INFO("Drive in %i, lastDirection = %i", *it, lastDirection);
        switch (lastDirection) {
        case RIGHT:
            switch (*it) {
            case RIGHT:
                break;
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
            case UP:
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
            case LEFT:
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
            case DOWN:
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        basicMovements.driveWall(0.80);
        lastDirection = *it;
    }
    basicMovements.stop();
    res.success = true;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle nh;


    // TODO: align to a cell
    Env env;
    env.alignToGrid();

    ros::ServiceServer service = nh.advertiseService("execute_plan", execute_plan_callback);
    ROS_INFO("ExecutePlan Service is ready.");

    ros::spin();
    return 0;
}
