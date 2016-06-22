#ifndef PATH_FINDER_LIB
#define PATH_FINDER_LIB

#include <constants.cpp>
#include <maze.cpp>
#include <position.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

class PathFinder
{
    public:
        PathFinder(std::vector<Row> rows)
        {
            // Initialize map in form of rows from
            // which columns are accessible
            this->rows = rows;
        }
        
        std::vector<int> find(Position start, Position end);

    private:
        // Variables
        bool DEBUG = true;
        std::vector<Row> rows; // Map variable
        std::vector<std::vector<int>> weightedMap;
        
        // External libraries

        // Functions
        void initializeWeightedMap(Position);
        void setDistancesInWeightedMap(Position);
        std::vector<Position> getNeighbours(Position);
};

std::vector<int> PathFinder::find(Position start, Position end)
{
    if(DEBUG) {
        ROS_INFO("PathFinder find start: %d end: %d", start.getXCoordinate(), start.getYCoordinate());
    }
    // Initialize weights in map starting from given position
    initializeWeightedMap(start);
    setDistancesInWeightedMap(start);

}

void PathFinder::initializeWeightedMap(Position start)
{
    for(int y; y < rows.size(); y++) {
        for(int x; x < rows[y].cells.size(); x++) {
            if(y == start.getYCoordinate() && x == start.getXCoordinate()) {
                weightedMap[x][y] = 0;
                continue;
            }
            weightedMap[x][y] = MAX_INT;
        }
    }
}

void PathFinder::setDistancesInWeightedMap(Position start)
{
    std::vector<Position> positionsBacklog;
    getNeighbours(start);
}

std::vector<Position> PathFinder::getNeighbours(Position position)
{
    std::vector<int> walls = rows[position.getYCoordinate()].cells[position.getXCoordinate()].walls;
    // Loop through all possible orientations to
    // discover if a neighbor is accessible or not.
    for (int i = 0; i < 4; i++) {
        // switch (walls[]) {

        // }
    }

    if (true) {

    }
}

#endif // PATH_FINDER_LIB
