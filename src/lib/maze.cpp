#ifndef MAZE_LIB
#define MAZE_LIB

// Needed includes for this library to work
#include "ros/ros.h"
// External libraries
#include <constants.cpp>
#include <wall_recognition.cpp>
#include <basic_movements.cpp>
#include <position.cpp>
#include <environment.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

using namespace orange_fundamentals;

class Maze
{
  public:
    Maze()
    {
        // env.align();
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
    // void updatePositionOnMap();

    std::vector<int> scanCurrentCell();
    std::vector<int> scanCurrentCellInitial();

    // TODO Should be private, public for testing
    std::vector<Position> initializePositions();
    bool compareWalls(Position possiblePosition, std::vector<int> wallsRobot);
    std::vector<Position> findPossiblePositions(std::vector<Position> positionsVector, std::vector<int> wallsRobotView);
    // keeps map
    std::vector<Row> rows;

  private:
    // External libraries
    BasicMovements basic_movements;
    WallRecognition wall_recognition;
    Env env;
    Position position{0, 0, 0}; // Position of robot


    ros::NodeHandle n;

    std::vector<Position> findPossibleCellsInitial();
    std::vector<Position> findPossibleCells();
    std::vector<int> rotateCellWallsClockwise(std::vector<int> wallsRobotView);

    void parseMap();
    void mapCallback(const Grid::ConstPtr& msg); // get map from service
    bool compareWallsInitial(std::vector<int> wallsMaze);
};

std::vector<Row> Maze::getMap() { return rows; }

void Maze::localize()
{
    std::vector<Position> possiblePositions = initializePositions();

    std::vector<int> wallsRobotView = scanCurrentCellInitial();

    possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);

    // loop solange bis nur noch eine possiblePosition existiert
    //
    //   fahre roboter
    //   updatePositionOnMap()
    //
    //   vergleiche sicht des roboters mit possiblePosition
    //
    // Setze position des Roboters auf letzte Position
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
    while (rows.size() == 0) {
        ros::spinOnce();
    }
    return;
}

// update the global @rows vector
void Maze::mapCallback(const Grid::ConstPtr& msg) { rows = msg->rows; }

std::vector<Position> Maze::findPossiblePositions(std::vector<Position> positionsVector,
                                                  std::vector<int> wallsRobotView)
{
    std::vector<Position> positionsLeft;
    for (int position = 0; position < positionsVector.size(); position++) {
        if (compareWalls(positionsVector[position], wallsRobotView)) {
            positionsLeft.push_back(positionsVector[position]);
        }
    }
    return positionsLeft;
}

bool Maze::compareWalls(Position possiblePosition, std::vector<int> wallsRobot)
{
    std::vector<int> wallOfCellBeforeDirection = rows[possiblePosition.getYCorrdinate()].cells[possiblePosition.getXCorrdinate()].walls;
    std::vector<int> wallOfCell;

    if (possiblePosition.getDirection() == UP) {
        wallOfCell = wallOfCellBeforeDirection;
    } else if (possiblePosition.getDirection() == RIGHT) {
        for(int i = 0; i < wallOfCellBeforeDirection.size(); i++){
            int newDirection = wallOfCellBeforeDirection[i]+1;
            if(newDirection > 3){
                newDirection = newDirection - 4;
            }
            wallOfCell.push_back(newDirection);
        }
    } else if (possiblePosition.getDirection() == LEFT) {
        for(int i = 0; i < wallOfCellBeforeDirection.size(); i++){
            int newDirection = wallOfCellBeforeDirection[i]-1;
            if(newDirection < 0){
                newDirection = newDirection + 4;
            }
            wallOfCell.push_back(newDirection);
        }
    } else if (possiblePosition.getDirection() == DOWN) {
        for(int i = 0; i < wallOfCellBeforeDirection.size(); i++){
            int newDirection = wallOfCellBeforeDirection[i]+2;
            if(newDirection > 3){
                newDirection = newDirection - 4;
            }
            wallOfCell.push_back(newDirection);
        }
    }
    for(int i = 0; i < wallOfCell.size(); i++){
        ROS_INFO("wallOfCell = %d", wallOfCell[i]);
    }

    if(wallOfCell.size() != wallsRobot.size()){
        return false;
    }

    for(int i = 0; i < wallsRobot.size(); i++){
        if (std::find(wallOfCell.begin(), wallOfCell.end(), wallsRobot[i]) == wallOfCell.end()) {
            return false;
        }
    }
    return true;
}

bool Maze::compareWallsInitial(std::vector<int> wallsMaze)
{
    // TODO
    return true;
}

std::vector<int> Maze::rotateCellWallsClockwise(std::vector<int> wallsRobotView)
{
    std::vector<int> newWallView;
    for (int i = 0; i < wallsRobotView.size(); i++) {
        switch (wallsRobotView[i]) {
        case RIGHT:
            newWallView.push_back(DOWN);
            break;
        case UP:
            newWallView.push_back(RIGHT);
            break;
        case LEFT:
            newWallView.push_back(UP);
            break;
        case DOWN:
            newWallView.push_back(LEFT);
            break;
        default:
            break;
        }
    }
    return newWallView;
}

std::vector<Position> Maze::findPossibleCellsInitial()
{
    std::vector<Position> possiblePositions = initializePositions();
    std::vector<int> wallsRobotView = scanCurrentCellInitial();

    // compareWalls(possiblePositions, wallsRobotView);

    for (int i = 0; i < rows.size(); i++) {
        for (int j = 0; j < rows[i].cells.size(); j++) {
            compareWallsInitial(rows[i].cells[i].walls);
        }
    }
    return {position};
}

std::vector<Position> Maze::initializePositions()
{
    std::vector<Position> positions{};
    for (int y = 0; y < rows.size(); y++) {
        for (int x = 0; x < rows[y].cells.size(); x++) {
            for (int direction = 0; direction < 4; direction++) {
                Position position{x, y, direction};
                positions.push_back(position);
            }
        }
    }
    return positions;
}

std::vector<Position> Maze::findPossibleCells() { return {position}; }

std::vector<int> Maze::scanCurrentCellInitial()
{
    // walls robot can see
    std::vector<int> wallsRobotView;
    std::vector<Wall*> walls = wall_recognition.getWalls();
    if (wall_recognition.hasFrontWall(walls)) {
        Wall frontWall = *wall_recognition.getFrontWall(walls);
        if(frontWall.isConfirmed()){
            wallsRobotView.push_back(DOWN);
        }
    }
    basic_movements.rotateBackwards();
    std::vector<int> currentCell = scanCurrentCell();
    wallsRobotView.insert(wallsRobotView.end(), currentCell.begin(), currentCell.end());
    return wallsRobotView;
}

std::vector<int> Maze::scanCurrentCell()
{
    std::vector<Wall*> walls = wall_recognition.getWalls();
    std::vector<int> wallsRobotView;
    if (wall_recognition.hasLeftWall(walls)) {
        Wall leftWall = *wall_recognition.getLeftWall(walls);
        if(leftWall.isConfirmed()){
            wallsRobotView.push_back(LEFT);
        }
    }
    if (wall_recognition.hasFrontWall(walls)) {
        Wall frontWall = *wall_recognition.getFrontWall(walls);
        if(frontWall.isConfirmed()){
            wallsRobotView.push_back(UP);
        }
    }
    if (wall_recognition.hasRightWall(walls)) {
        Wall rightWall = *wall_recognition.getRightWall(walls);
        if(rightWall.isConfirmed()){
            wallsRobotView.push_back(RIGHT);
        }
    }
    return wallsRobotView;
}

#endif // MAZE_LIB
