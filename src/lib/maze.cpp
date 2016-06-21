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
    std::vector<int> scanCurrentCellInitial();

  private:

    // Position of robot
    Position *position = new Position(0,0,0);

    // keeps map
    std::vector<Row> rows;

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
    std::vector<Position> possiblePositions = initializePositions();

    std::vector<int> wallsRobotView = scanCurrentCellInitial();

    // vergleiche sicht des roboters mit possiblePosition
    // possiblePosition wird dezimiert
    
    scanCurrentCell();

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
    std::vector<Position> possiblePositions = initializePositions();
    std::vector<int> wallsRobotView = scanCurrentCellInitial();

    compareWalls(possiblePositions, wallsRobotView);

    for (int i = 0; i < rows.size(); i++) {
        for (int j = 0; j < rows[i].cells.size(); j++) {
            compareWallsInitial(rows[i].cells[i].walls);
        }
    }
    return {*position};
}

std::vector<Position> Maze::initializePositions()
{
    std::vector<Position> positions = new std::vector<Position>();
    Position position;
    for(int y = 0; y < rows.size(); y++) {
        for(int x = 0; x < rows[y].size(); x++) {
            for(int direction = 0; direction < rows.size(); direction++) {
                position = new Position(x,y,direction);
                positions.push_back(position);
            }
        }
    }
    return positions;
}

std::vector<Position> Maze::findPossibleCells()
{
    return {*position};
}

std::vector<int> Maze::scanCurrentCellInitial(std::vector<int> wallsMaze) 
{
    // walls robot can see
    // std::vector<Wall*> wallsRobotView;
    std::vector<int> wallsRobotView;
    std::vector<Wall*> walls = wall_recognition.getWalls();
    if (wall_recognition.hasFrontWall(walls)) {
        wallsRobotView.push_back(BOTTOM);
    }
    basic_movements.rotateBackwards();
    std::vector<int> currentCell = scanCurrentCell();
    wallsRobotView.insert(wallsRobotView.end(), currentCell.begin(), currentCell.end());
    return wallsRobotView;
}

std::vector<int> Maze::scanCurrentCell() {
    std::vector<Wall*> walls = wall_recognition.getWalls();
    std::vector<int> wallsRobotView;
    if (wall_recognition.hasLeftWall(walls)) {
        wallsRobotView.push_back(LEFT);
    }
    if (wall_recognition.hasFrontWall(walls)) {
        wallsRobotView.push_back(UP);
    }
    if (wall_recognition.hasRightWall(walls)) {
        wallsRobotView.push_back(RIGHT);
    }
    return wallsRobotView;

}






#endif // MAZE_LIB
