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
        // Vector with cells to explore
        std::vector<Position> neighbourBacklog;
        Position currentCell;
        
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
    neighbourBacklog = {};
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

/*
* Calculate all distances from starting point to all 
* posible destinations. Breadth-first search is used as
* algorithm.
*
* Parameters: 
*   currentCell - start position from where all distances are
*                 calculated
*/
void PathFinder::setDistancesInWeightedMap(Position currentCell)
{
    // Get all reachable cells from current position
    std::vector<Position> neighbours = getNeighbours(currentCell);
    // Append all neighbours to vectors with unexplored cells
    neighbourBacklog.insert(neighbourBacklog.end(), neighbours.begin(), neighbours.end());
    int weightCurrentCell = weightedMap[currentCell.getXCoordinate()][currentCell.getYCoordinate()];
    // Select next neighbour in breadth first search
    Position neighbourOfInterest = neighbourBacklog[0];
    // Weight next neighbour
    int weightNeighbourOfInterest = weightedMap[neighbourBacklog[neighbourOfInterest].getXCoordinate()][neighbourBacklog[neighbourOfInterest].getYCoordinate()];
    // Update weight of neighbour
    if (weightCurrentCell++ < weightNeighbourOfInterest ) {
        weightedMap[neighbourBacklog[neighbourOfInterest].getXCoordinate()][neighbourBacklog[neighbourOfInterest].getYCoordinate()] = weightCurrentCell++;
    }
    // Erase explored neighbour
    neighbourBacklog.erase(neighbourBacklog.begin());

    // Check if there is a next neighbour to explore
    if (neighbourBacklog.size() == 0) { 
        return; 
    }
    // Continue to explore the rest of the map
    setDistancesInWeightedMap(neighbourOfInterest);
}

std::vector<Position> PathFinder::getNeighbours(Position position)
{
    std::vector<int> walls = rows[position.getYCoordinate()].cells[position.getXCoordinate()].walls;
    // std::vector<> v;
    // Loop through all possible orientations to
    // discover if a neighbor is accessible or not.
    for (int i = 0; i < 4; i++) {
        // switch (walls[i]) {
        //     case TOP:
        // }
    }

    if (true) {

    }
}

#endif // PATH_FINDER_LIB
