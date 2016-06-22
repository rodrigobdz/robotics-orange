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
        if (!DEBUG) {
            env.align();
            parseMap();
            localize();
        } else {
            ROS_INFO("align start");
            env.align();
            ROS_INFO("align finished, start parse");
            parseMap();
            ROS_INFO("parse finished, start localize");
            localize();
            ROS_INFO("localize finished");
        }
    }

    void localize();

    Position getPosition();

    std::vector<Position> updatePositionsForward(std::vector<Position> positions);
    std::vector<Position> updatePositionsTurn(std::vector<Position> positions, int turn);

  private:
    // Variables
    ros::NodeHandle n;
    std::vector<Row> rows;
    Position position{0, 0, 0}; // Position of robot

    std::vector<Position> possiblePositions;
    std::vector<Wall*> walls;
    std::vector<int> wallsRobotView;

    // External libraries
    BasicMovements basic_movements;
    WallRecognition wall_recognition;
    Env env;

    bool DEBUG = true;

    std::vector<Position> findPossibleCells();

    void turnToFreeFrontWall();

    std::vector<int> scanCurrentCell();
    std::vector<int> scanCurrentCellInitial();

    std::vector<Position> initializePositions();
    std::vector<Position> findPossiblePositions(std::vector<Position> positionsVector, std::vector<int> wallsRobotView);
    bool compareWalls(Position possiblePosition, std::vector<int> wallsRobot);
    void searchCorrectPosition();

    void parseMap();
    void mapCallback(const Grid::ConstPtr& msg); // get map from service
};

Position Maze::getPosition() { return position; }

void Maze::localize()
{
    while (possiblePositions.size() > 1 || possiblePositions.size() == 0) {
        possiblePositions = initializePositions();
        wallsRobotView = scanCurrentCellInitial();
        possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);
        walls = wall_recognition.getWalls();

        if (DEBUG) {
            ROS_INFO("Start looking for free front");
        }
        turnToFreeFrontWall();
        if (DEBUG) {
            ROS_INFO("Finished looking for free front");
            ROS_INFO("Start looking for correct position");
        }

        if (DEBUG) {
            ROS_INFO("Current possiblePosition = %lu", possiblePositions.size());
            for (int i = 0; i < possiblePositions.size(); i++) {
                ROS_INFO("X = %d, Y = %d, direction = %d", possiblePositions[i].getXCoordinate(),
                         possiblePositions[i].getYCoordinate(), possiblePositions[i].getDirection());
            }
        }
        searchCorrectPosition();
    }

    position = possiblePositions[0];
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
    map.shutdown();
    return;
}

// TODO Annahme bearbeiten
// Annahme ist dass die Positionsbewegung auch Sinn macht
std::vector<Position> Maze::updatePositionsForward(std::vector<Position> positions)
{
    std::vector<Position> updatedPositions;

    // Drive forward
    for (int i = 0; i < positions.size(); i++) {
        if (positions[i].getDirection() == UP) {
            int newX = positions[i].getXCoordinate();
            int newY = positions[i].getYCoordinate() - 1;
            int newDirection = positions[i].getDirection();
            if (newX > -1 && newX < rows[0].cells.size()) {
                if (newY > -1 && newY < rows.size()) {
                    updatedPositions.push_back(Position{newX, newY, newDirection});
                }
            }
        } else if (positions[i].getDirection() == LEFT) {
            int newX = positions[i].getXCoordinate() - 1;
            int newY = positions[i].getYCoordinate();
            int newDirection = positions[i].getDirection();
            if (newX > -1 && newX < rows[0].cells.size()) {
                if (newY > -1 && newY < rows.size()) {
                    updatedPositions.push_back(Position{newX, newY, newDirection});
                }
            }
        } else if (positions[i].getDirection() == RIGHT) {
            int newX = positions[i].getXCoordinate() + 1;
            int newY = positions[i].getYCoordinate();
            int newDirection = positions[i].getDirection();
            if (newX > -1 && newX < rows[0].cells.size()) {
                if (newY > -1 && newY < rows.size()) {
                    updatedPositions.push_back(Position{newX, newY, newDirection});
                }
            }
        } else if (positions[i].getDirection() == DOWN) {
            int newX = positions[i].getXCoordinate();
            int newY = positions[i].getYCoordinate() + 1;
            int newDirection = positions[i].getDirection();
            if (newX > -1 && newX < rows[0].cells.size()) {
                if (newY > -1 && newY < rows.size()) {
                    updatedPositions.push_back(Position{newX, newY, newDirection});
                }
            }
        }
    }
    return updatedPositions;
}

std::vector<Position> Maze::updatePositionsTurn(std::vector<Position> positions, int turn)
{
    std::vector<Position> updatedPositions;
    for (int i = 0; i < positions.size(); i++) {
        if (turn == UP) {
            updatedPositions.push_back(
                Position{positions[i].getXCoordinate(), positions[i].getYCoordinate(), positions[i].getDirection()});
        } else if (turn == RIGHT) {
            int newDirection = positions[i].getDirection() - 1;
            if (newDirection < 0) {
                newDirection = newDirection + 4;
            }
            updatedPositions.push_back(
                Position{positions[i].getXCoordinate(), positions[i].getYCoordinate(), newDirection});
        } else if (turn == LEFT) {
            int newDirection = positions[i].getDirection() + 1;
            if (newDirection > 3) {
                newDirection = newDirection - 4;
            }
            updatedPositions.push_back(
                Position{positions[i].getXCoordinate(), positions[i].getYCoordinate(), newDirection});
        } else if (turn == DOWN) {
            int newDirection = positions[i].getDirection() + 2;
            if (newDirection > 3) {
                newDirection = newDirection - 4;
            }
            updatedPositions.push_back(
                Position{positions[i].getXCoordinate(), positions[i].getYCoordinate(), newDirection});
        }
    }
    return updatedPositions;
}

// update the global @rows vector
void Maze::mapCallback(const Grid::ConstPtr& msg) { rows = msg->rows; }

std::vector<Position> Maze::findPossiblePositions(std::vector<Position> positionsVector,
                                                  std::vector<int> wallsRobotView)
{
    std::vector<Position> positionsLeft;
    ROS_INFO("1.6.1");
    for (int position = 0; position < positionsVector.size(); position++) {
        ROS_INFO("1.6.2");
        if (compareWalls(positionsVector[position], wallsRobotView)) {
            positionsLeft.push_back(positionsVector[position]);
            ROS_INFO("1.6.3");
        }
    }
    ROS_INFO("1.6.4");
    if (DEBUG) {
        if (positionsLeft.size() == 0) {
            ROS_INFO("PositionsLeft 0");
        }
    }
    return positionsLeft;
}

bool Maze::compareWalls(Position possiblePosition, std::vector<int> wallsRobot)
{
    std::vector<int> wallOfCellBeforeDirection =
        rows[possiblePosition.getYCoordinate()].cells[possiblePosition.getXCoordinate()].walls;
    std::vector<int> wallOfCell;

    if (possiblePosition.getDirection() == UP) {
        wallOfCell = wallOfCellBeforeDirection;
    } else if (possiblePosition.getDirection() == RIGHT) {
        for (int i = 0; i < wallOfCellBeforeDirection.size(); i++) {
            int newDirection = wallOfCellBeforeDirection[i] + 1;
            if (newDirection > 3) {
                newDirection = newDirection - 4;
            }
            wallOfCell.push_back(newDirection);
        }
    } else if (possiblePosition.getDirection() == LEFT) {
        for (int i = 0; i < wallOfCellBeforeDirection.size(); i++) {
            int newDirection = wallOfCellBeforeDirection[i] - 1;
            if (newDirection < 0) {
                newDirection = newDirection + 4;
            }
            wallOfCell.push_back(newDirection);
        }
    } else if (possiblePosition.getDirection() == DOWN) {
        for (int i = 0; i < wallOfCellBeforeDirection.size(); i++) {
            int newDirection = wallOfCellBeforeDirection[i] + 2;
            if (newDirection > 3) {
                newDirection = newDirection - 4;
            }
            wallOfCell.push_back(newDirection);
        }
    }

    if (wallOfCell.size() != wallsRobot.size()) {
        return false;
    }

    for (int i = 0; i < wallsRobot.size(); i++) {
        if (std::find(wallOfCell.begin(), wallOfCell.end(), wallsRobot[i]) == wallOfCell.end()) {
            return false;
        }
    }
    return true;
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
    if (DEBUG) {
        ROS_INFO("Starting with %lu positions", positions.size());
    }
    return positions;
}

std::vector<Position> Maze::findPossibleCells() { return {position}; }

std::vector<int> Maze::scanCurrentCellInitial()
{
    // walls robot can see
    std::vector<int> wallsRobotView;
    walls = wall_recognition.getWalls();
    if (wall_recognition.hasFrontWall(walls)) {
        Wall frontWall = *wall_recognition.getFrontWall(walls);
        if (frontWall.isConfirmed()) {
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
    walls = wall_recognition.getWalls();
    std::vector<int> wallsRobotView;
    if (wall_recognition.hasLeftWall(walls)) {
        Wall leftWall = *wall_recognition.getLeftWall(walls);
        if (leftWall.isConfirmed()) {
            wallsRobotView.push_back(LEFT);
        }
    }
    if (wall_recognition.hasFrontWall(walls)) {
        Wall frontWall = *wall_recognition.getFrontWall(walls);
        if (frontWall.isConfirmed()) {
            wallsRobotView.push_back(UP);
        }
    }
    if (wall_recognition.hasRightWall(walls)) {
        Wall rightWall = *wall_recognition.getRightWall(walls);
        if (rightWall.isConfirmed()) {
            wallsRobotView.push_back(RIGHT);
        }
    }
    return wallsRobotView;
}

void Maze::turnToFreeFrontWall()
{
    walls = wall_recognition.getWalls();
    while (wall_recognition.hasFrontWall(walls)) {
        if (!wall_recognition.getFrontWall(walls)->isConfirmed()) {
            break;
        }
        basic_movements.rotateRight();
        possiblePositions = updatePositionsTurn(possiblePositions, RIGHT);
        walls = wall_recognition.getWalls();
    }
}

void Maze::searchCorrectPosition()
{
    while (possiblePositions.size() > 1) {
        Wall* rightWall = NULL;
        Wall* frontWall = NULL;
        Wall* leftWall = NULL;

        walls = wall_recognition.getWalls();

        ROS_INFO("1");
        if (!wall_recognition.hasRightWall(walls)) {
            ROS_INFO("1.1");
            basic_movements.rotateRight();
            ROS_INFO("1.2");
            possiblePositions = updatePositionsTurn(possiblePositions, RIGHT);
            ROS_INFO("1.3");
            basic_movements.driveWall(0.8);
            ROS_INFO("1.4");
            possiblePositions = updatePositionsForward(possiblePositions);
            ROS_INFO("1.5");
            std::vector<int> wallsRobotView = scanCurrentCell();
            ROS_INFO("1.6");
            possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);
            ROS_INFO("1.7");
            continue;
        } else {
            ROS_INFO("3");
            rightWall = wall_recognition.getRightWall(walls);
            if (!rightWall->isConfirmed()) {
                basic_movements.rotateRight();
                possiblePositions = updatePositionsTurn(possiblePositions, RIGHT);
                basic_movements.driveWall(0.8);
                possiblePositions = updatePositionsForward(possiblePositions);
                std::vector<int> wallsRobotView = scanCurrentCell();
                possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);
                continue;
            }
        }

        if (!wall_recognition.hasFrontWall(walls)) {
            ROS_INFO("4");
            basic_movements.driveWall(0.8);
            possiblePositions = updatePositionsForward(possiblePositions);
            std::vector<int> wallsRobotView = scanCurrentCell();
            possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);
            continue;
        } else {
            ROS_INFO("5");
            frontWall = wall_recognition.getFrontWall(walls);
            if (!frontWall->isConfirmed()) {
                ROS_INFO("1");
                basic_movements.driveWall(0.8);
                possiblePositions = updatePositionsForward(possiblePositions);
                std::vector<int> wallsRobotView = scanCurrentCell();
                possiblePositions = findPossiblePositions(possiblePositions, wallsRobotView);
                continue;
            }
        }

        if (!wall_recognition.hasLeftWall(walls)) {
            ROS_INFO("6");
            basic_movements.rotateLeft();
            possiblePositions = updatePositionsTurn(possiblePositions, LEFT);
            basic_movements.driveWall(0.8);
            possiblePositions = updatePositionsForward(possiblePositions);
            continue;
        } else {
            ROS_INFO("7");
            leftWall = wall_recognition.getLeftWall(walls);
            if (!leftWall->isConfirmed()) {
                ROS_INFO("8");
                basic_movements.rotateLeft();
                possiblePositions = updatePositionsTurn(possiblePositions, LEFT);
                basic_movements.driveWall(0.8);
                possiblePositions = updatePositionsForward(possiblePositions);
                continue;
            }
        }

        ROS_INFO("9");
        basic_movements.rotateBackwards();
        possiblePositions = updatePositionsTurn(possiblePositions, DOWN);
        basic_movements.driveWall(0.8);
        possiblePositions = updatePositionsForward(possiblePositions);
    }
}
#endif // MAZE_LIB
