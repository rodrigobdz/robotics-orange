#include "ros/ros.h"
#include <cstdlib>

#include "orange_fundamentals/ExecutePlan.h"
#include <basic_movements.cpp>
#include <environment.cpp>

Position currentPosition;

bool executePlanCallback(orange_fundamentals::MoveToPosition::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    Position nearestPosition;
    std::vector<int> shortestPath;

    req.bow;
    req.column;

    res.


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle nh;

    Plan plan;
    Maze maze;
    PathFinder pathFinder;

    ros::ServiceServer service = nh.advertiseService("move_to_position", executePlanCallback);
    ROS_INFO("Move to position service is ready.");

    ros::spin();

    return 0;
}
