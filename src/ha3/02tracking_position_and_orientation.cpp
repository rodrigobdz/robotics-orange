#include "ros/ros.h"
#include "orange_fundamentals/ExecutePlan.h"
#include <basic_movements.cpp>
#include <environment.cpp>
#include <cstdlib>
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "orange_fundamentals/Pose.h"

enum directions { RIGHT = 0, UP, LEFT, DOWN };

// TODO: Values will be updated from localize function
int currentPosition = 0;
int currentColumn = 0;
int currentOrientation = 0;

void publishPositionAndOrientation(int row, int column, int orientation) {
    Pose msg;
    msg.row = row;
    msg.column column;
    msg.orientation = orientation;

    pose.publish(msg);
    ros::spinOnce();

    ros::Rate r(1);
    r.sleep();
}

bool executePlanCallback(orange_fundamentals::ExecutePlan::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    BasicMovements basicMovements;
    std::vector<int> plan = req.plan;
    res.success           = true;
    int lastOrientation   = UP;

    publishPositionAndOrientation(row, column, lastOrientation);

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {

        ROS_INFO("execute_plan_callback: %d", *it);
        ROS_INFO("Drive in %i, lastOrientation = %i", *it, lastOrientation);

        switch (lastOrientation) {
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
        basicMovements.driveWall(0.80);
        lastOrientation = *it;
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle nh;
    
    ros::Publisher pose = nh.advertise<Pose>("pose", 1000);

    // TODO: Import localize function
    localization();

    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlanCallback);
    ROS_INFO("ExecutePlan Service is ready.");

    ros::spin();

    return 0;
}