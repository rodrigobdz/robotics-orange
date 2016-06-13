#include "ros/ros.h"
#include "orange_fundamentals/ExecutePlan.h"
#include <basic_movements.cpp>
#include <environment.cpp>
#include <cstdlib>
#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "orange_fundamentals/Pose.h"

using namespace orange_fundamentals;

ros::Publisher pose;
enum directions { RIGHT = 0, UP, LEFT, DOWN };

// TODO: Values will be updated from localize function
// TODO: Current orientation must be UP
int currentRow         = 0;
int currentColumn      = 0;
int currentOrientation = 0;

// TODO: Implement this function
void localize(void){}

void publishPositionAndOrientation(int row, int column, int orientation) {
    Pose msg;
    msg.row         = row;
    msg.column      = column;
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

    publishPositionAndOrientation(currentRow, currentColumn, currentOrientation);

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {

        ROS_INFO("execute_plan_callback: %d", *it);

        switch (currentOrientation) {
        case RIGHT:
            switch (*it) {
            case UP:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case LEFT:
                res.success = res.success && basicMovements.rotate(180);
                break;
            case DOWN:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            default:
                break;
            }
            break;
        case UP:
            switch (*it) {
            case RIGHT:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            case LEFT:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case DOWN:
                res.success = res.success && basicMovements.rotate(180);
                break;
            default:
                break;
            }
            break;
        case LEFT:
            switch (*it) {
            case RIGHT:
                res.success = res.success && basicMovements.rotate(180);
                break;
            case UP:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            case DOWN:
                res.success = res.success && basicMovements.rotate(90);
                break;
            default:
                break;
            }
            break;
        case DOWN:
            switch (*it) {
            case RIGHT:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case UP:
                res.success = res.success && basicMovements.rotate(180);
                break;
            case LEFT:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
        res.sucess = basicMovements.driveWall(CELL_LENGTH);
        currentOrientation = *it;
        publishPositionAndOrientation(currentRow, currentColumn, currentOrientation);
    }

    return res.success;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "execute_plan_server");
    ros::NodeHandle nh;
    
    pose = nh.advertise<Pose>("pose", 1000);

    // TODO: Import localize function
    localize();

    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlanCallback);
    ROS_INFO("ExecutePlan Service is ready.");

    ros::spin();

    return 0;
}