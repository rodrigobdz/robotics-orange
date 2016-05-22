#include "ros/ros.h"
#include "orange_fundamentals/ExecutePlan.h"
#include <basic_movements.cpp>

enum directions { RIGHT = 0, UP, LEFT, DOWN };


bool execute_plan_callback(orange_fundamentals::ExecutePlan::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    BasicMovements basicMovements;
    std::vector<int> plan = req.plan;
    res.success = true;
    int lastDirection = UP;

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        
        ROS_INFO("execute_plan_callback: %d", *it);
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

        res.success = basicMovements.drive(CELL_LENGTH);
        lastDirection = *it;
    }
    
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle nh;

    // TODO: align to a cell

    ros::ServiceServer service = nh.advertiseService("execute_plan", execute_plan_callback);
    ROS_INFO("ExecutePlan Service is ready.");

    ros::spin();
    return 0;
}
