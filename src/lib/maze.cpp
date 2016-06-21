#ifndef MAZE_LIB
#define MAZE_LIB

// Needed includes for this library to work
#include "ros/ros.h"
#include <cstdlib>
#include <constants.cpp>
#include <wall_recognition.cpp>
#include <basic_movements.cpp>
#include <position.cpp>
//#include <cell.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

using namespace orange_fundamentals;

class Maze
{
  public:
    Maze()
    {
        parseMap();
        // localize();
    }

    // returns map
    std::vector<Row> getMap();

    void localize();

    // TODO Implement getPosition
    // Position getPosition();

    // TODO Need better name to differ from real move to theoretically move
    // Maybe boolean to differ between move forward and backward
    // Unneccesary for the first implementation
    // void moveOnMap();

    std::vector<int> scanCurrentCell();

  private:

    // direction of robot
    int direction = true;

    // Position of robot
    Position *position = new Position(0,0,0);

    // keeps map
    std::vector<Row> rows;

    // walls robot can see
    // std::vector<Wall*> wallsRobotView;
    std::vector<int> wallsRobotView;

    // External libraries
    BasicMovements basicMovements;
    WallRecognition wall_recognition;

    // get map from service
    void parseMap();

    ros::NodeHandle n;
    void mapCallback(const Grid::ConstPtr& msg);

    std::vector<Position> findPossibleCellsInitial();
    std::vector<Position> findPossibleCells();
    bool compareWallsInitial(std::vector<int> wallsMaze);
    bool compareWalls(std::vector<int> wallsMaze);



    // wallpattern getWallPattern(std::vector<int> walls);
    // std::string wallsToString(std::vector<int> v);
    // bool filter90d(const Wall* w);
    // bool contains(std::vector<int> v, int e);
    // std::vector<int> scanCurrentCell_test();
};

std::vector<Row> Maze::getMap()
{
    return rows;
}

void Maze::localize()
{
    std::vector<Position> possiblePositions;
    
    scanCurrentCell();

    possiblePositions = findPossibleCells();


}

/************************************************************/
/*                         Helpers                          */
/************************************************************/

/**
 *  Builds up the local representation of the maze
 * */
void Maze::parseMap()
{
    // get map from service
    ros::Subscriber map;
    map = n.subscribe("map", 1, &Maze::mapCallback, this);
    while(rows.size() == 0){
        ros::spinOnce();
    }
    return;
}

// update the global @rows vector
void Maze::mapCallback(const Grid::ConstPtr& msg)
{
    rows = msg->rows;
}

std::vector<Position> Maze::findPossibleCellsInitial()
{
    std::vector<Position> possiblePositions;
    for (int i = 0; i < rows.size(); i++) {
        for (int j = 0; j < rows[i].cells.size(); j++) {
            compareWallsInitial(rows[i].cells[i].walls);
        }
    }
    return {*position};
}

std::vector<Position> Maze::findPossibleCells()
{
    return {*position};
}

bool Maze::compareWallsInitial(std::vector<int> wallsMaze) {
    // looks to the right
    if (wall_recognition.hasLeftWall(wall_recognition.getWalls())) {

    }
}

std::vector<int> Maze::scanCurrentCell() {
    return {LEFT, UP};

}






#endif // MAZE_LIB
