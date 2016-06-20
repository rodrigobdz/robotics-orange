#ifndef MAZE_LIB
#define MAZE_LIB

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>
#include "create_fundamentals/DiffDrive.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include "create_fundamentals/ResetEncoders.h"
#include <constants.cpp>
#include <wall_recognition.cpp>
#include <math.h>
#include "basic_movements.cpp"
#include <Cell.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"


#define DEBUG true

class Maze
{
  public:
    Maze() {
        // TODO This function needs to list to the map topic and retrieve it
        parseMap();
        localize();
    }

    // TODO Need better name to differ from real move to theoretically move
    // Maybe boolean to differ between move forward and backward
    // Unneccesary for the first implementation
    void moveOnMap();
    // TODO Better generic name for left
    // Boolean to differ between right and left turn
    void turnOnMap(bool left);

    void localize();
    //TODO Implement getPosition
    //Position getPosition();

  private:
    // TODO Implement position class, maybe connect with cell
    // Position robot;

    // get map from
    void parseMap();

    std::vector<int> scanCurrentCell();

    // wallpattern getWallPattern(std::vector<int> walls);
    std::string     wallsToString(std::vector<int> v);
    bool            filter90d(const Wall* w);
    bool            contains(std::vector<int> v, int e);
    std::vector<int> scanCurrentCell_test();
    void mapCallback(const Grid::ConstPtr& msg);
};

// update the global @rows vector
void Maze::mapCallback(const Grid::ConstPtr& msg) { rows = msg->rows; }



void Maze::getMap()
{

}


/**
 *  TODO needs refactoring
 *  Builds up the local representation of the maze
 * */
void Maze::parseMap()
{
    // get map from service
    ros::init(argc, argv, "initialize parse map");
    ros::NodeHandle n;
    
    ros::Subscriber map;
    map = n.subscribe("map", 1, &mapCallback);

    // if (rows.empty()) {
    //     if (DEBUG) { ROS_INFO("parseMap: Error - rows vector is empty"); }
    //     return;
    // }

    // std::vector<Cell> cells;
    // std::vector<int> walls;

    // if (DEBUG) {
    //     ROS_INFO("parseMap: READ IN THE CURRENT MAP");
    // }
        
    // // matrix[i][j]
    // for (int i = 0; i < rows.size(); i++) {
    //     cells = rows[i];
    //     for (int j = 0; j < cells.size(); j++) {
    //         walls = cells[j];


    //     }
        
    // }


    laserSubscriber     = n.subscribe("scan_filtered", 1, &BasicMovements::laserCallback, this);



    return;





    // for (std::vector<Row>::iterator irow = rows.begin(); irow != rows.end(); ++irow) {
    //     x = (x + 1) % DIMENSION; // increment row index
    //     cells = (*irow).cells;   // cells of the row[x]

    //     for (std::vector<Cell>::iterator icell = cells.begin(); icell != cells.end(); ++icell) {
    //         y = (y + 1) % DIMENSION; // increment column index
    //         walls = (*icell).walls;  // walls of cell[x][y]

    //         // setup a cell structure, fill in all informations
    //         std::pair<int, int> coord = std::make_pair(x, y);
    //         struct cell* c = &maze[INDEX(coord)];
    //         c->coordinates = coord;
    //         c->walls = walls;
    //         std::sort(c->walls.begin(), c->walls.end());
    //         c->pattern = getWallPattern(c->walls);

    //         if (DEBUG)
    //             ROS_INFO("parseMap: (%d,%d) -> %s (%s)", c->coordinates.first, c->coordinates.second,
    //                      wallsToString(c->walls).c_str(), wp_lookup[c->pattern].c_str());
    //     }
    // }
}













































// /**
//  *  Returns the current position of the robot.
//  * */
// Position getPosition(){
//     return position;
// }

// void turnOnMap(bool left){
//     return;

// }

// // global pointer to the current map
// std::vector<Row> rows;

// std::vector<int> scanCurrentCell(void)
// {
//     WallRecognition rs;
//     BasicMovements bm;
//     std::vector<int> walls;
//     int orientation = (int)TOP;

//     // rotate four times 90 degrees
//     for (int i = 0; i < 4; i++) {
//         std::vector<Wall*> w = rs.getWalls();

//         // erase all walls from w which aren't almost at 0 degrees
//         // that means just hold the wall right in front
//         // if there is one
//         w.erase(std::remove_if(w.begin(), w.end(), [](const Wall* _w) { return not filter90d(_w); }), w.end());

//         if (w.size() > 0)
//             walls.push_back(TOP);

//         bm.rotateRight();
//         ROTATEALL(walls);
//     }

//     if (DEBUG)
//         ROS_INFO("scanCurrentCell: walls.size() = %d, %s", (int)walls.size(), wallsToString(walls).c_str());

//     std::sort(walls.begin(), walls.end());

//     if (contains(walls, TOP)) {
//         if (getWallPattern(walls) == end) {
//             bool blocked = true;
//             while (blocked) {
//                 bm.rotateLeft();
//                 ROTATEALL(walls);
//                 orientation++;
//                 if (!(walls.at(TOP) == TOP))
//                     blocked = false;
//             }
//         }

//         if (getWallPattern(walls) == corner) {
//             ROS_INFO("is corner.");
//             if (contains(walls, RIGHT)) {
//                 bm.rotateLeft();
//                 ROTATEALL(walls);
//                 ROTATEALL(walls);
//                 ROTATEALL(walls);
//                 orientation++;
//             } else {
//                 bm.rotateRight();
//                 ROTATEALL(walls);
//                 orientation--;
//             }
//         }

//         if (getWallPattern(walls) == wall or getWallPattern(walls) == path) {
//             ROS_INFO("is wall or path.");
//             bm.rotateLeft();
//             ROTATEALL(walls);
//             orientation++;
//         }
//     }

//     orientation = orientation % 4;
//     walls.push_back(orientation);

//     // debug
//     ROS_INFO("scanCurrentCell: walls.size() = %d, %s", (int)walls.size(), wallsToString(walls).c_str());
//     return walls;
// }

// /* macro EQUAL: return true if @w1 and @w2 contain the same elements */
// #define EQUAL(w1, w2) ((w1).size() == (w2).size() && std::equal((w1).begin(), (w1).end(), (w2).begin()))

// /*
//  * TODO Needs refactoring
//  * Localize the robot to the given map.
//  * */
// int* localize(void)
// {
//     // hold possible positions
//     // 0: x coordinate of cell
//     // 1: y coordinate of cell
//     // 2: orientation
//     std::vector<int*> pos;
//     std::vector<int*> tmp;

//     // at the beginning every cell is a possible location, therefore
//     // generate a position in @pos for each cell in the maze
//     for (int i = 0; i < DIMENSION * DIMENSION; i++) {
//         pos.push_back(new int[3]);
//         pos.back()[0] = maze[i].coordinates.first;
//         pos.back()[1] = maze[i].coordinates.second;
//         pos.back()[2] = TOP; // initial orientation
//     }

//     // scanCurrentCell should return an integer vector of size n
//     // where n is at most equal to 4, the n's member is a direction which
//     // is the orientation of the Robot, the first (n-1) are although directions
//     // which are scanned walls
//     std::vector<int> curr_w;
//     directions orientation;
//     BasicMovements bm;

//     bool first_run = true;
//     while (pos.size() > 1) {
//         curr_w = scanCurrentCell();
//         orientation = (directions)curr_w.back();
//         curr_w.pop_back();

//         // iterate over all positions left
//         for (std::vector<int*>::iterator ipos = pos.begin(); ipos != pos.end(); ++ipos) {
//             std::vector<int> ipos_w = maze[INDEX(std::make_pair((*ipos)[0], (*ipos)[1]))].walls;

//             directions ipos_o;
//             if (first_run)
//                 ipos_o = TOP;
//             else
//                 ipos_o = (directions)(*ipos)[2];

//             for (int i = 0; i < 4; i++) {
//                 if (EQUAL(curr_w, ipos_w)) {
//                     if (DEBUG)
//                         ROS_INFO("localize: got possiblity (%d,%d)|%c ->w %s", (*ipos)[0], (*ipos)[1],
//                                  directions_lookup[ipos_o], wallsToString(ipos_w).c_str());

//                     tmp.push_back(new int[3]);
//                     tmp.back()[0] = (*ipos)[0];
//                     tmp.back()[1] = (*ipos)[1];
//                     tmp.back()[2] = ipos_o;
//                     break;
//                 }

//                 ROTATE_R(ipos_o);
//                 ROTATEALL(ipos_w);
//                 std::sort(ipos_w.begin(), ipos_w.end());

//                 if (DEBUG)
//                     ROS_INFO("localize: -->r curr_w: %s|%c", wallsToString(ipos_w).c_str(), directions_lookup[ipos_o]);
//             }
//         } // end iterator over positions

//         pos.clear();
//         pos.swap(tmp);
//         tmp.clear();

//         if (DEBUG)
//             ROS_INFO("localize: pos.size()=%d", (int)pos.size());

//         if (counter == 2 or pos.size() == 1) {
//             if (DEBUG)
//                 ROS_INFO("localize: last possible position - pos[0]=(%d,%d,%c)", pos.at(0)[0], pos.at(0)[1],
//                          directions_lookup[pos.at(0)[2]]);
//             break;
//         }

//         std::for_each(pos.begin(), pos.end(), move);

// #if 1 // really move the robot
//         bm.driveWall(CELL_LENGTH);
// #endif

//         first_run = false;
//     }

//     return pos.at(0);
// }

// /*-------------------------helper-functions-------------------------*/

// /* @return a matching wallpattern for the given @walls vector. */
// wallpattern getWallPattern(std::vector<int> walls)
// {
//     if (walls.empty()) {
//         if (DEBUG)
//             ROS_INFO("getWallPattern: Error - parameter 'walls' is empty");
//         return failure;
//     }

//     switch (walls.size()) {
//     case 3:
//         return end;
//     case 1:
//         return wall;
//     case 2:
//         if (abs(walls.at(0) + walls.at(1)) % 2 == 0)
//             return path;
//         else
//             return corner;

//     default:
//         if (DEBUG)
//             ROS_INFO("getWallPattern: Error - walls.size() = %d is not in {1,2,3}", (int)walls.size());
//         return failure;
//     }
// }

// std::string wallsToString(std::vector<int> v)
// {
//     int bytesWritten = 0; // pointer to the current write position in buffer
//     char buffer[v.size() * 2 + 2];

//     bytesWritten += snprintf(buffer, sizeof(buffer), "[");
//     for (std::vector<int>::iterator iv = v.begin(); iv != v.end(); ++iv) {
//         bytesWritten += snprintf(buffer + bytesWritten, sizeof(buffer), "%c", directions_lookup[*iv]);
//         if (std::next(iv) != v.end())
//             bytesWritten += snprintf(buffer + bytesWritten, sizeof(buffer), ",");
//     }
//     snprintf(buffer + bytesWritten, sizeof(buffer), "]");
//     return std::string(buffer);
// }

// bool filter90d(const Wall* w)
// {
//     // if(DEBUG) ROS_INFO("filterWall: w.getAngleInRadians() = %f", w->getAngleInRadians());
//     /* OLD APPROCH, FIXME: delete if the new code is tested
//     */
//     if (w->getAngleInRadians() < 0.2 && w->getAngleInRadians() > -0.2) {
//         // ROS_INFO("filterWall: return true;");
//         return true;
//     }

//     return false;

//     // return (w->getAngleInRadians() < 0.1 && w->getAngleInRadians() > -0.1);
// }

// bool contains(std::vector<int> v, int e)
// {
//     for (std::vector<int>::iterator iv = v.begin(); iv != v.end(); ++iv) {
//         if ((*iv) == e)
//             return true;
//     }
//     return false;
// }

// std::vector<int> scanCurrentCell_test()
// {
//     int w[3];
//     switch (counter) {
//     case 0:
//         w[0] = TOP;
//         w[1] = LEFT;
//         w[2] = TOP;
//         counter++;
//         break;
//     case 1:
//         w[0] = TOP;
//         w[1] = LEFT;
//         w[2] = TOP;
//         counter++;
//         break;
//     }
//     return std::vector<int>(w, w + 3);
// }

#endif // MAZELIB
