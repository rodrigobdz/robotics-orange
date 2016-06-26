#include "ros/ros.h"
#include <cstdlib>

#include "orange_fundamentals/MoveToPosition.h"
#include <basic_movements.cpp>
#include <environment.cpp>
#include <position.cpp>
#include <maze.cpp>
#include <path_finder.cpp>
#include <plan.cpp>

using namespace orange_fundamentals;

Position currentPosition;

bool executePlanCallback(orange_fundamentals::MoveToPosition::Request& req,
                           orange_fundamentals::MoveToPosition::Response& res)
{

    int goalX = req.column;
    int goalY = req.row;

    res.success       = true;

    Position goalPosition{goalX, goalY, -1};

    Maze maze{currentPosition};

    PathFinder pathFinder;
    std::vector<int> plan_instructions = pathFinder.find(currentPosition, goalPosition);
 
    Plan plan;
    plan.execute(plan_instructions, currentPosition.getDirection());

    maze.updatePositionOnMap(plan_instructions);
    currentPosition = maze.getPosition();

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_to_position_server");
    ros::NodeHandle nh;

    Maze maze;

    currentPosition = maze.getPosition();

    ros::ServiceServer service = nh.advertiseService("move_to_position", executePlanCallback);
    ROS_INFO("Move to position service is ready.");

    ros::spin();

    return 0;
}
