
#include "ros/ros.h"
#include "orange_fundamentals/ExecutePlan.h"

enum directions { RIGHT = 0, UP, LEFT, DOWN };

bool execute_plan_callback(orange_fundamentals::ExecutePlan::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    std::vector<int> plan = req.plan;
    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {
        ROS_INFO("execute_plan_callback: %d", *it);

        switch (*it) {
        case RIGHT:
            break;
        case UP:
            break;
        case LEFT:
            break;
        case DOWN:
            break;
        default:
            break;
        }
    }
    res.success = true;
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
