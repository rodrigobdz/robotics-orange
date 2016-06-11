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
#include <ransac.cpp>

/* /src/lib/maze.h - all maze (map) related stuff */
#include <maze.h>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"
//#include "orange_fundamentals/ExecutePlan.h"
using namespace orange_fundamentals;

#define DEBUG true
#define not_implemented_yet -2

enum directions { RIGHT = 0, TOP, LEFT, BOTTOM };

// global pointer to the current map
std::vector<Row> rows;


/** ------------------------function-prototypes------------------------- */

/* n-dimensional vector of integers to string */
std::string WallsToString(std::vector<int> v);

/* filter function, @return true if the angle to wall @w is almost 0 degrees
   used to check if there is a wall right in front of the robot */
bool filter90d(const Wall* w);

/* get and set the wallpattern of @c */
wallpattern getWallPattern(std::vector<int> walls);

/** -------------------------------end---------------------------------- */


// just update the global @rows vector
void mapCallback(const Grid::ConstPtr& msg)
{
    rows = msg->rows;
}

// build up the local representation of the maze
void parseMap(void)
{
    if(rows.empty()) {
        if(DEBUG) ROS_INFO("parseMap: Error - rows vector is empty -> mapCallback hasn't run yet");
        return;
    }

    int i = -1, j = -1;
    std::vector<Cell> cells;
    std::vector<int>  walls;

    if(DEBUG) ROS_INFO("parseMap: READ IN THE CURRENT MAP");

    for(std::vector<Row>::iterator irow = ::rows.begin();
            irow != ::rows.end(); ++irow) {

        i = (i+1)%DIMENSION;   // increment row index
        cells = (*irow).cells; // cells of the row[i]

        for(std::vector<Cell>::iterator icell = cells.begin();
                icell != cells.end(); ++icell) {

            j = (j+1)%DIMENSION;    // increment column index
            walls = (*icell).walls; // walls of cell[i][j]

            // setup a cell structure, fill in all informations
            std::pair<int,int> coord = std::make_pair(i,j);
            struct cell c = maze[INDEX(coord)];
            c.coordinates = coord;
            c.walls = walls;
            std::sort(c.walls.begin(), c.walls.end());
            c.pattern = getWallPattern(c.walls);

            if(DEBUG) ROS_INFO("parseMap: (%d,%d) -> %s (%s)",
                               c.coordinates.first,
                               c.coordinates.second,
                               WallsToString(c.walls).c_str(),
                               wp_lookup[c.pattern].c_str());
        }
    }
}

/* @return a matching wallpattern for the given @walls vector. */
wallpattern getWallPattern(std::vector<int> walls)
{
    if(walls.empty()) {
        if(DEBUG) ROS_INFO("getWallPattern: Error - parameter 'walls' is empty");
        return failure;
    }

    switch(walls.size()) {
    case 3: return end;
    case 1: return wall;
    case 2:
        if(abs(walls.at(0) + walls.at(1)) % 2 == 0)
            return path;
        else
            return corner;

    default:
        if(DEBUG) ROS_INFO("getWallPattern: Error - walls.size() = %d is not in {1,2,3}",
                           (int) walls.size());
        return failure;
    }
}

// FIXME: not tested!
std::vector<int> scanCurrentCell(void)
{
    int i = 0; // count the rotations
    Ransac rs;
    BasicMovements bm;

    std::vector<int> walls;

    // rotate four times 90 degrees
    for(i; i<4; i++) {
        std::vector<Wall*> w = rs.getWalls();

        // erase all walls from w which aren't almost at 0 degrees
        // that means just hold the wall right in front
        // if there is one
        w.erase(
                std::remove_if(w.begin(), w.end(), [](const Wall* _w) {return not filter90d(_w);}),
                w.end()
                );
        if(w.size() > 0)
            walls.push_back(i);
        bm.rotate(90);
    }

    // get back to initial orientation
    bm.rotate(90);

    // debug
    //ROS_INFO("scanCurrentCell: walls.size() = %d", (int) walls.size());
    //ROS_INFO("scanCurrentCell: 0:%d, 1:%d", walls.at(0), walls.at(1));
    return walls;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_localization");
    ros::NodeHandle nh;

    // Align the robot to the center of a cell
    // TODO: fix the align function
    //Env env;
    //env.alignToGrid();

    ros::Subscriber map;
    map = nh.subscribe("map", 1, &mapCallback);

    // read the map from /map publisher
    // therefore wait a moment and call spinOnce
    // ? have to call it two times, why ?
    ros::Rate r(1);
    r.sleep();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    parseMap();
    //scanCurrentCell();

    // TODO: publish the map coordinates and orientation to /pose
    // TODO: find the robots current location in the map

    //ros::ServiceServer service = nh.advertiseService("execute_plan", executePlanCallback);
    //ROS_INFO("ExecutePlan Service is ready.");

    return 0;
}


/** ------------------------helper-functions------------------------- */


bool filter90d(const Wall* w)
{
    if(DEBUG) ROS_INFO("filterWall: w.getAngle() = %f", w->getAngle());
    /* OLD APPROCH, FIXME: delete if the new code is tested
    if(w->getAngle() < 0.1 && w->getAngle() > -0.1)
        return true;
    else
        return false;
    */

    return (w->getAngle() < 0.1 && w->getAngle() > -0.1);
}

std::string WallsToString(std::vector<int> v)
{
    int bytesWritten = 0; // pointer to the current write position in buffer
    char directions_lookup[] = { 'R', 'T', 'L', 'B' };
    char buffer[v.size()*2 + 2];

    bytesWritten += snprintf(buffer, sizeof(buffer), "[");
    for(std::vector<int>::iterator iv = v.begin(); iv != v.end(); ++iv) {
        bytesWritten += snprintf(buffer+bytesWritten, sizeof(buffer), "%c", directions_lookup[*iv]);
        if(std::next(iv) != v.end())
            bytesWritten += snprintf(buffer+bytesWritten, sizeof(buffer), ",");
    }
    snprintf(buffer+bytesWritten, sizeof(buffer), "]");
    return std::string (buffer);
}
