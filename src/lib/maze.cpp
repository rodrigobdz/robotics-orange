
#include "./maze.h"

#define DEBUG true

// helper functions
wallpattern getWallPattern(std::vector<int> walls);
std::string WallsToString(std::vector<int> v);
bool filter90d(const Wall* w);
bool contains(std::vector<int> v, int e);
std::vector<int> scanCurrentCell_test(void);

int counter = 0;

/* move: @param position is an integer array with length 3, which
   fields are 0: x coordinate in the maze, 1: y coordinate in the maze,
   2: orientation. The function changes the coordinates in place according
   to an move in the given direction. */
void move(int* position)
{
    switch(position[2]) {
    case RIGHT:  position[1] += 1; break;
    case TOP:    position[0] -= 1; break;
    case LEFT:   position[1] -= 1; break;
    case BOTTOM: position[0] += 1; break;
    default: ;
    }
}

using namespace orange_fundamentals;

// build up the local representation of the maze
void parseMap(std::vector<Row> _rows)
{
    if(_rows.empty()) {
        if(DEBUG) ROS_INFO("parseMap: Error - rows vector is empty -> mapCallback hasn't run yet");
        return;
    }

    int i = -1, j = -1;
    std::vector<Cell> cells;
    std::vector<int>  walls;

    if(DEBUG) ROS_INFO("parseMap: READ IN THE CURRENT MAP");

    for(std::vector<Row>::iterator irow = _rows.begin();
            irow != _rows.end(); ++irow) {

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

#include "basic_movements.cpp"
#include "ransac.cpp"

std::vector<int> scanCurrentCell(void)
{
    Ransac rs;
    BasicMovements bm;
    std::vector<int> walls;
    int orientation = (int) TOP;

    // rotate four times 90 degrees
    for(int i=0; i<4; i++) {
        std::vector<Wall*> w = rs.getWalls();

        // erase all walls from w which aren't almost at 0 degrees
        // that means just hold the wall right in front
        // if there is one
        w.erase(
                std::remove_if(w.begin(), w.end(), [](const Wall* _w) {return not filter90d(_w);}),
                w.end()
                );

        if(w.size() > 0)
            walls.push_back(TOP);

        bm.rotate(-90);
        ROTATEALL(walls);
    }

    if(DEBUG) ROS_INFO("scanCurrentCell: walls.size() = %d, %s",
                       (int) walls.size(),
                       WallsToString(walls).c_str());

    std::sort(walls.begin(), walls.end());

    if(contains(walls, TOP)) {

        if(getWallPattern(walls) == end) {
            bool blocked = true;
            while(blocked) {
                bm.rotate(90);
                //ROTATEALL(walls);
                orientation++;
                if(!(walls.at(TOP) == TOP))
                    blocked = false;
            }
        }

        if(getWallPattern(walls) == corner) {
            ROS_INFO("is corner.");
            if(contains(walls, RIGHT)) {
                bm.rotate(90);
                //ROTATEALL(walls);
                //ROTATEALL(walls);
                //ROTATEALL(walls);
                orientation++;
            } else {
                bm.rotate(-90);
                //ROTATEALL(walls);
                orientation--;
            }
        }

        if(getWallPattern(walls) == wall or getWallPattern(walls) == path) {
            ROS_INFO("is wall or path.");
            bm.rotate(90);
            //ROTATEALL(walls);
            orientation++;
        }
    }

    orientation = orientation%4;
    walls.push_back(orientation);

    // debug
    ROS_INFO("scanCurrentCell: walls.size() = %d, %s",
             (int) walls.size(),
             WallsToString(walls).c_str());
    return walls;
}

/* macro EQUAL: return true if @w1 and @w2 contain the same elements */
#define EQUAL(w1,w2) ((w1).size() == (w2).size() && std::equal((w1).begin(), (w1).end(), (w2).begin()))

int* localize(void)
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

    bool first_run = true;
    while(pos.size()>1) {
        curr_w = scanCurrentCell_test();
        orientation = (directions) curr_w.back();
        curr_w.pop_back();

        // iterate over all positions left
        for(std::vector<int*>::iterator ipos = pos.begin(); ipos != pos.end(); ++ipos) {

            std::vector<int> ipos_w =
                maze[INDEX(std::make_pair((*ipos)[0],(*ipos)[1]))].walls;

            directions ipos_o;
            if(first_run)
                ipos_o = TOP;
            else
                ipos_o = (directions)(*ipos)[2];

            for(int i=0; i<4; i++) {

                if(EQUAL(curr_w,ipos_w)) {
                    if(DEBUG) ROS_INFO("localize: got possiblity (%d,%d)|%c ->w %s",
                                       (*ipos)[0],
                                       (*ipos)[1],
                                       directions_lookup[ipos_o],
                                       WallsToString(ipos_w).c_str());

                    tmp.push_back(new int[3]);
                    tmp.back()[0] = (*ipos)[0];
                    tmp.back()[1] = (*ipos)[1];
                    //tmp.back()[2] = ipos_o;
                    tmp.back()[2] = orientation;
                    break;
                }

                //ROTATE_R(ipos_o);
                ROTATEALL(ipos_w);
                std::sort(ipos_w.begin(), ipos_w.end());

                if(DEBUG) ROS_INFO("localize: -->r curr_w: %s|%c",
                                   WallsToString(ipos_w).c_str(),
                                   directions_lookup[ipos_o]);
            }
        } // end iterator over positions

        pos.clear();
        pos.swap(tmp);
        tmp.clear();

        if(DEBUG) ROS_INFO("localize: pos.size()=%d",
                           (int) pos.size());

        if(counter == 2 or pos.size() == 1) {
            if(DEBUG) ROS_INFO("localize: last possible position - pos[0]=(%d,%d,%c)",
                                pos.at(0)[0],
                                pos.at(0)[1],
                                directions_lookup[pos.at(0)[2]]);
            break;
        }

        std::for_each(pos.begin(),pos.end(), move);

#if 0 // really move the robot
        bm.driveWall(0.8);
#endif

        first_run = false;
    }

    return pos.at(0);
}

/*-------------------------helper-functions-------------------------*/


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

bool filter90d(const Wall* w)
{
    //if(DEBUG) ROS_INFO("filterWall: w.getAngle() = %f", w->getAngle());
    /* OLD APPROCH, FIXME: delete if the new code is tested
    */
    if(w->getAngle() < 0.2 && w->getAngle() > -0.2) {
        //ROS_INFO("filterWall: return true;");
        return true;
    }

    return false;

    //return (w->getAngle() < 0.1 && w->getAngle() > -0.1);
}

bool contains(std::vector<int> v, int e)
{
    for(std::vector<int>::iterator iv = v.begin(); iv != v.end(); ++iv) {
        if((*iv) == e) return true;
    }
    return false;
}

std::vector<int> scanCurrentCell_test()
{
    int w[3];
    switch(counter) {
    case 0:
        w[0] = RIGHT;
        w[1] = TOP;
        w[2] = BOTTOM;
        counter++;
        break;
    case 1:
        w[0] = TOP;
        w[1] = LEFT;
        w[2] = RIGHT;
        counter++;
        break;
    }
    return std::vector<int>(w, w+3);
}
