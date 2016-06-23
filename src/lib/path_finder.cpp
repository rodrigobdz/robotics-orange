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
        PathFinder()
        {
            // Initialize map in form of rows from
            // which columns are accessible
            parseMap();
        }
        
        std::vector<int> find(Position start, Position end);
        void initializeWeightedMap(Position);
        void setDistancesInWeightedMap(Position, Position);
        std::vector<Position> getNeighbours(Position);

        std::vector<std::vector<int>> weightedMap;

    private:
        // Variables
        bool DEBUG = true;
        std::vector<Row> rows; // Map variable
        ros::NodeHandle n;
        
        // Vector with cells to explore
        std::vector<Position> neighbourBacklog;
        Position currentCell;
        std::vector<std::vector<Position>> possiblePaths;
        
        // External libraries

        // Functions
        void parseMap();
        void mapCallback(const Grid::ConstPtr& msg); // get map from service
};

std::vector<int> PathFinder::find(Position start, Position end)
{
    if(DEBUG) {
        ROS_INFO("PathFinder find start: %d end: %d", start.getXCoordinate(), start.getYCoordinate());
    }
    // Initialize weights in map starting from given position
    initializeWeightedMap(start);
    setDistancesInWeightedMap(start, end);

}

void PathFinder::initializeWeightedMap(Position start)
{
    for(int x = 0; x < rows[0].cells.size(); x++) {
        std::vector<int> column;
        for(int y = 0; y < rows.size(); y++) {
        
            if(y == start.getYCoordinate() && x == start.getXCoordinate()) {

                column.push_back(0);
                continue;
            }
            column.push_back(MAX_INT);
        }
        weightedMap.push_back(column);
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
void PathFinder::setDistancesInWeightedMap(Position currentCell, Position goalCell)
{
    // Get all reachable cells from current position
    std::vector<Position> neighbours = getNeighbours(currentCell);
    ROS_INFO("Test2");

    // Append all neighbours to vectors with unexplored cells
    neighbourBacklog.insert(neighbourBacklog.end(), neighbours.begin(), neighbours.end());
    int weightCurrentCell = weightedMap[currentCell.getXCoordinate()][currentCell.getYCoordinate()];
    ROS_INFO("Test3");
    
    // Select next neighbour in breadth first search
    Position neighbourOfInterest = neighbourBacklog[0];
    ROS_INFO("Test4");
    
    // Weight next neighbour
    int weightNeighbourOfInterest = weightedMap[neighbourOfInterest.getXCoordinate()][neighbourOfInterest.getYCoordinate()];
    
    // Update weight of neighbour
    if (weightCurrentCell + 1 < weightNeighbourOfInterest ) {
        weightedMap[neighbourOfInterest.getXCoordinate()][neighbourOfInterest.getYCoordinate()] = weightCurrentCell + 1;
    }
    
    // Erase explored neighbour
    neighbourBacklog.erase(neighbourBacklog.begin());

    // Check if there is a next neighbour to explore
    if (neighbourBacklog.size() == 0) { 
        return; 
    }
    
    // Continue to explore the rest of the map
    setDistancesInWeightedMap(neighbourOfInterest, goalCell);
}

std::vector<Position> PathFinder::getNeighbours(Position position)
{
    ROS_INFO("TestNeighbourBefore");
    ROS_INFO("size %lu %lu", rows[0].cells.size(), rows.size());
    std::vector<int> walls = rows[position.getYCoordinate()].cells[position.getXCoordinate()].walls;
    ROS_INFO("TestNeighbourAfter");
    std::vector<Position> neighbours;
    Position possibleNeighbour;

    // std::vector<> v;
    // Loop through all possible orientations to
    // discover if a neighbor is accessible or not.
    for (int i = 0; i < 4; i++) {
        ROS_INFO("TestNeighbour");
        if (std::find(walls.begin(),walls.end(),i) == walls.end()) {
            ROS_INFO("TestNeighbourT %i", i);
            if(i == RIGHT){
                possibleNeighbour = {position.getXCoordinate() + 1, position.getYCoordinate(), -1};
            } else if(i == UP){
                possibleNeighbour = {position.getXCoordinate(), position.getYCoordinate() - 1, -1};
            }else if(i == LEFT){
                possibleNeighbour = {position.getXCoordinate() - 1, position.getYCoordinate(), -1};
            }else if(i == DOWN){
                possibleNeighbour = {position.getXCoordinate(), position.getYCoordinate() + 1, -1};
            }
            neighbours.push_back(possibleNeighbour);    
        }
    }
    return neighbours;
}

void PathFinder::parseMap()
{
    // get map from service
    ros::Subscriber map;
    map = n.subscribe("map", 1, &PathFinder::mapCallback, this);
    while (rows.size() == 0) {
        ros::spinOnce();
    }
    map.shutdown();
    return;
}

// update the global @rows vector
void PathFinder::mapCallback(const Grid::ConstPtr& msg) { rows = msg->rows; }



#endif // PATH_FINDER_LIB
