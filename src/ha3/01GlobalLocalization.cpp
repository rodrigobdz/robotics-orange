/* Robotics Fundamentals Ex3 - Task 1 - global localization in the maze
   @author: group Orange
*/
// emacs-lisp:
// (setq tab-width 4)
// (setq c-basic-offset 4)
// (setq-default c-basic-offset 4)

// import standart libraries
#include <utility>
#include <iterator>
#include <string>

#include "ros/ros.h"

// import our libraries
#include <basic_movements.cpp>
#include <environment.cpp>
#include <constants.cpp>
#include <ransac.cpp>
#include <play_song.cpp>

#include "create_fundamentals/PlaySong.h"
#include "create_fundamentals/StoreSong.h"
#include "orange_fundamentals/ExecutePlan.h"

/* /src/lib/maze.h - all maze (map) related stuff */
#include <maze.h>

#define DEBUG true

// global pointer to the current map
std::vector<Row> rows;

ros::Publisher pose;

// TODO: Values will be updated from localize function
// TODO: Current orientation must be UP
int currentRow         = 0;
int currentColumn      = 0;
int currentOrientation = 0;

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

/*
    Rotate the robot to look upward regarding the map
*/
void rotateUP()
{
    BasicMovements basicMovements;
    switch(currentOrientation) {
        case RIGHT:
            basicMovements.rotate(90);
            break;
        case LEFT:
            basicMovements.rotate(-90);
        case BOTTOM:
            basicMovements.rotate(180);
            break;
        default:
            break;
    }
    currentOrientation = TOP;
}

bool executePlanCallback(orange_fundamentals::ExecutePlan::Request& req,
                           orange_fundamentals::ExecutePlan::Response& res)
{
    BasicMovements basicMovements;
    PlaySongLib playSongLib;

    std::vector<int> plan = req.plan;
    res.success           = true;

    publishPositionAndOrientation(currentRow, currentColumn, currentOrientation);

    for (std::vector<int>::iterator it = plan.begin(); it != plan.end(); ++it) {

        ROS_INFO("execute_plan_callback: %d", *it);

        switch (currentOrientation) {
        case RIGHT:
            switch (*it) {
            case TOP:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case LEFT:
                res.success = res.success && basicMovements.rotate(180);
                break;
            case BOTTOM:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            default:
                break;
            }
            break;
        case TOP:
            switch (*it) {
            case RIGHT:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            case LEFT:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case BOTTOM:
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
            case TOP:
                res.success = res.success && basicMovements.rotate(-90);
                break;
            case BOTTOM:
                res.success = res.success && basicMovements.rotate(90);
                break;
            default:
                break;
            }
            break;
        case BOTTOM:
            switch (*it) {
            case RIGHT:
                res.success = res.success && basicMovements.rotate(90);
                break;
            case TOP:
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
        res.success = res.success && basicMovements.driveWall(CELL_LENGTH);
        currentOrientation = *it;
        // TODO: Tripel hier als argument übergeben.
        //move(moveArg);
        publishPositionAndOrientation(currentRow, currentColumn, currentOrientation);

        if(res.success == false) {
            playSongLib.starWars();
            Env env;
            env.alignToGrid();
            localize();
            // Prepare for next plan execution with UP orientation
            rotateUP();
            return res.success;
        }
    }

    return res.success;
}

// just update the global @rows vector
void mapCallback(const Grid::ConstPtr& msg)
{
    rows = msg->rows;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_localization");
    ros::NodeHandle nh;
    ros::Rate r(1);


    // Align the robot to the center of a cell
    // TODO: fix the align function
#if 0
    Env env;
    env.alignToGrid();
#endif

    /* GET AND PARSE THE MAP */
#if 1
    ros::Subscriber map;
    map = nh.subscribe("map", 1, &mapCallback);

    // read the map from /map publisher
    // therefore wait a moment and call spinOnce
    // ? have to call it two times, why ?
    r.sleep();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    parseMap(rows);
    localize();

#endif

    //scanCurrentCell();

#if 0
    int* pos = localize();

    ros::Publisher pose = nh.advertise<Pose>("pose", 1000);
    ros::ServiceServer service = nh.advertiseService("execute_plan", executePlanCallback);

    while(ros::ok()) {
        Pose msg;
        msg.row = pos[0];
        msg.column = pos[1];
        msg.orientation = pos[2];

        pose.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
#endif

    return 0;
}
