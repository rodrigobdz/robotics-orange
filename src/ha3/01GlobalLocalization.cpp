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

/* /src/lib/maze.h - all maze (map) related stuff */
#include <maze.h>

#define DEBUG true

// global pointer to the current map
std::vector<Row> rows;

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
