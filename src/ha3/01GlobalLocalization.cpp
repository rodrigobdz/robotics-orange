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

int counter = 0;

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
    int w[3];
    switch(counter) {
    case 0:
        w[0] = LEFT;
        w[1] = BOTTOM;
        w[2] = TOP;
        counter++;
        break;
    case 1:
        w[0] = TOP;
        w[1] = LEFT;
        w[2] = TOP;
        counter++;
        break;
    }
    return std::vector<int>(w, w+3);
#endif

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

/* macro EQUAL: return true if @w1 and @w2 contain the same elements */
#define EQUAL(w1,w2) ((w1).size() == (w2).size() && std::equal((w1).begin(), (w1).end(), (w2).begin()))

void localization(void)
{

    // hold possible positions
    // 0: x coordinate of cell
    // 1: y coordinate of cell
    // 2: orientation
    std::vector<int*> pos;
    std::vector<int*> tmp;

    // at the beginning every cell is a possible location, therefore
    // generate a position in @pos for each cell in the maze
    for(int i=0; i<DIMENSION*DIMENSION; i++) {
        pos.push_back(new int[3]);
        pos.back()[0] = maze[i].coordinates.first;
        pos.back()[1] = maze[i].coordinates.second;
        pos.back()[2] = TOP; // initial orientation
    }

    // scanCurrentCell should return an integer vector of size n
    // where n is at most equal to 4, the n's member is a direction which
    // is the orientation of the Robot, the first (n-1) are although directions
    // which are scanned walls
    std::vector<int> curr_w;
    directions orientation;
    BasicMovements bm;

    while(pos.size()>1) {
        curr_w = scanCurrentCell();
        orientation = (directions) curr_w.back();
        curr_w.pop_back();

        if(DEBUG) ROS_INFO("localization: curr_w: %s|%d",
                           WallsToString(curr_w).c_str(),
                           orientation);

        // iterate over all positions left
        for(std::vector<int*>::iterator ipos = pos.begin(); ipos != pos.end(); ++ipos) {

            if(DEBUG) ROS_INFO("localization: ipos: (%d,%d) %d",
                               (*ipos)[0],
                               (*ipos)[1],
                               (*ipos)[2]);

            std::vector<int> ipos_w =
                maze[INDEX(std::make_pair((*ipos)[0],(*ipos)[1]))].walls;

            //directions ipos_o = (directions)(*ipos)[2];
            directions ipos_o = TOP;

            if(DEBUG) ROS_INFO("localization: ipos_w: (%d,%d) %s",
                               (*ipos)[0],
                               (*ipos)[1],
                               WallsToString(ipos_w).c_str());

            for(int i=0; i<4; i++) {

                if(EQUAL(curr_w,ipos_w)) {
                    if(DEBUG) ROS_INFO("localization: pos: (%d,%d)|%c ->w %s",
                                       (*ipos)[0],
                                       (*ipos)[1],
                                       directions_lookup[ipos_o],
                                       WallsToString(ipos_w).c_str());

                    tmp.push_back(new int[3]);
                    tmp.back()[0] = (*ipos)[0];
                    tmp.back()[1] = (*ipos)[1];
                    tmp.back()[2] = ipos_o;
                    break;
                }

                ROTATE_R(ipos_o);
                ROTATEALL(ipos_w);
                std::sort(ipos_w.begin(), ipos_w.end());

                if(!DEBUG) ROS_INFO("localization: -->r curr_w: %s|%c",
                                   WallsToString(ipos_w).c_str(),
                                   directions_lookup[ipos_o]);
            }
        } // end iterator over positions

        if(DEBUG) ROS_INFO("localization: pos.size()=%d, tmp.size()=%d, o=%d",
                       (int) pos.size(),
                       (int) tmp.size(),
                       orientation);

        pos.clear();
        pos.swap(tmp);
        tmp.clear();

        if(counter == 2) {
            ROS_INFO("pos[0]=(%d,%d,%d)", pos.at(0)[0],pos.at(0)[1],pos.at(0)[2]);
            break;
        }

        std::for_each(pos.begin(),pos.end(), move);
#if 0 // really move the robot
        bm.driveWall(0.8);
#endif
    }

    if(DEBUG) ROS_INFO("localization: pos.size()=%d, tmp.size()=%d, o=%d",
                       (int) pos.size(),
                       (int) tmp.size(),
                       orientation);
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
