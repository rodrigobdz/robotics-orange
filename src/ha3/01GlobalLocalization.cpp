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
#include <constants.cpp>

/* /src/lib/maze.h - all maze (map) related stuff */
#include <maze.h>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

//#include "orange_fundamentals/ExecutePlan.h"
using namespace orange_fundamentals;

#define DEBUG true
#define not_implemented_yet -2

// global pointer to the current map
std::vector<Row> rows;


/** ------------------------function-prototypes------------------------- */

/* n-dimensional vector of integers to string */
std::string WallsToString(std::vector<int> v);

/* filter function, @return true if the angle to wall @w is almost 0 degrees
   used to check if there is a wall right in front of the robot */
bool filter90d(const Wall* w);

/* get the wallpattern of @walls */
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
            struct cell* c = &maze[INDEX(coord)];
            c->coordinates = coord;
            c->walls = walls;
            std::sort(c->walls.begin(), c->walls.end());
            c->pattern = getWallPattern(c->walls);

            if(DEBUG) ROS_INFO("parseMap: (%d,%d) -> %s (%s)",
                               c->coordinates.first,
                               c->coordinates.second,
                               WallsToString(c->walls).c_str(),
                               wp_lookup[c->pattern].c_str());
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
// TODO: change function to meet following expectations
//   The function should return a vector with directions which
//   correspond to scanned walls and the robots orientation.
//   Additionally the robot should not face a wall, its orientation
//   should be a direction were its free to move. (note: if the robot
//   previously just came to this cell, it should not be this direction
//   the robot faces, except there is no else direction to go)
// TODO: return walls sorted
std::vector<int> scanCurrentCell(void)
{
#if 1 // for testing
    int w[] = {LEFT,BOTTOM,TOP};
    return std::vector<int>(w, w+3);
#endif

    int i = 0; // count the rotations
    
    BasicMovements bm;

    std::vector<int> walls;

    // rotate four times 90 degrees
    for(i; i<4; i++) {

        // check if wall in front
        if(isWallInFront() {
            walls.push_back(TOP);
        }
        bm.rotate(PI/2);
        bm.ROTATEALL(walls);
    }

    std::sort(walls.begin(), walls.end());
    
    if(walls.at(TOP) == TOP) {
            // robot in dead end
        if(getWallPattern(walls) == end) { //nicht in dead end reingefahren
            bool blocked = true;
            while (blocked) {
                bm.rotate(PI/2);
                bm.ROTATEALL(walls);
                if (!(walls.at(TOP) == TOP)) {
                    blocked = false;
                }
            }
        }

        // robot entered corner
        if(getWallPattern(walls) == corner) {
            if(walls.at(RIGHT) == RIGHT) {
                bm.rotate(-PI/2);
                bm.ROTATEALL(walls);
                bm.ROTATEALL(walls);
                bm.ROTATEALL(walls);
            } else {
                bm.rotate(PI/2);
                bm.ROTATEALL(walls);
            }
        }

        // wall in front of robot
        if(getWallPattern(walls) == wall) {
                bm.rotate(PI/2);
                bm.ROTATEALL(walls);
        }

    // debug
    //ROS_INFO("scanCurrentCell: walls.size() = %d", (int) walls.size());
    //ROS_INFO("scanCurrentCell: 0:%d, 1:%d", walls.at(0), walls.at(1));
    return walls;
}

/* macro EQUAL: return true if @w1 and @w2 contain the same elements */
#define EQUAL(w1,w2) (std::equal((w1).begin(), (w1).end(), (w2).begin()))

void localization(void)
{
    // hold all possible cells which could be
    // the current location; if P shrinks to |P|=1
    // we know excatly were we are;
    // at the beginning P is just a copy of the maze,
    // because every field is a possible location
    std::vector<cell> P (DIMENSION*DIMENSION);
    std::copy(maze, maze + (DIMENSION*DIMENSION), P.begin());

    // hold possible positions
    // 0: x coordinate of cell
    // 1: y coordinate of cell
    // 2: orientation
    std::vector<int*> pos;

    // debug
    //ROS_INFO("P.size()=%d", (int)P.size());
    //ROS_INFO("pattern: %d != %d", P.at(1).pattern, maze[1].pattern);

    // scanCurrentCell should return an integer vector of size n
    // where n is at most equal to 4, the n's member is a direction which
    // is the orientation of the Robot, the first (n-1) are although directions
    // which are scanned walls
    std::vector<int> curr;
    wallpattern wp;
    directions orientation;


    while(P.size()>1) {
        curr = scanCurrentCell();
        orientation = (directions) curr.back();
        curr.pop_back();

        if(DEBUG) ROS_INFO("localization: curr: %s", WallsToString(curr).c_str());

        // sort out all cells with a not matching wall pattern
        wp = getWallPattern(curr);
        FILTERPATTERN(P,wp);

        // iterate over all cells left
        for(std::vector<cell>::iterator ic = P.begin(); ic != P.end(); ++ic) {
            std::vector<int> curr_w = (*ic).walls;
            directions curr_o = TOP;

            if(DEBUG) ROS_INFO("localization: curr_w: (%d,%d) %s",
                               (*ic).coordinates.first,
                               (*ic).coordinates.second,
                               WallsToString(curr_w).c_str());

            for(int i=0; i<4; i++) {

                if(EQUAL(curr,curr_w)) {

                    pos.push_back(new int[3]);
                    pos.back()[0] = (*ic).coordinates.first;
                    pos.back()[1] = (*ic).coordinates.second;
                    pos.back()[2] = curr_o;

                    if(DEBUG) ROS_INFO("localization: pos: [%d,%d,%d]",
                                       pos.back()[0],
                                       pos.back()[1],
                                       pos.back()[2]);

                    move(pos.back());
                    break;
                }

                ROTATE_R(curr_o);
                ROTATEALL(curr_w);
                std::sort(curr_w.begin(), curr_w.end());

                if(DEBUG) ROS_INFO("localization: -->r curr_w: %s|%c",
                                   WallsToString(curr_w).c_str(),
                                   directions_lookup[curr_o]);
            }
        } // end iterator over P

        //P.clear();

        break;
    }

    if(DEBUG) ROS_INFO("localization: pos.size()=%d, curr.size()=%d, wp=%s, o=%d",
                       (int) pos.size(),
                       (int) curr.size(),
                       wp_lookup[wp].c_str(),
                       orientation);

    if(DEBUG) ROS_INFO("localization: P.size()=%d", (int)P.size());
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
    localization();
    //scanCurrentCell();

    // TODO: publish the map coordinates and orientation to /pose
    // TODO: find the robots current location in the map

    //ros::ServiceServer service = nh.advertiseService("execute_plan", executePlanCallback);
    //ROS_INFO("ExecutePlan Service is ready.");

    return 0;
}


/** ------------------------helper-functions------------------------- */


bool isWallInFront() {
    Ransac rs;
    std::vector<Wall*> w = rs.getWalls();
    for (int i=0; i<rs.size(); i++){
        if(rs[i]->getAngle < 0.1 && rs[i]->getAngle > -0.1) {
            return true;
        }
    }
    return false;
    if(DEBUG) ROS_INFO("filterWall: w.getAngle() = %f", rs->getAngle());
}

bool filter90d(const Wall* w)
{
    if(DEBUG) ROS_INFO("filterWall: w.getAngle() = %f", w->getAngle());
    //  OLD APPROCH, FIXME: delete if the new code is tested
    // if(w->getAngle() < 0.1 && w->getAngle() > -0.1)
    //     return true;
    // else
    //     return false;
    

    return (w->getAngle() < 0.1 && w->getAngle() > -0.1);
}

std::string WallsToString(std::vector<int> v)
{
    int bytesWritten = 0; // pointer to the current write position in buffer
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
