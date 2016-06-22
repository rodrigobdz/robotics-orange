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
        std::vector<std::vector<int>> weightedRows;
        
        // External libraries

        // Functions
        void initializeFloodMap(Position start);
};

std::vector<int> PathFinder::find(Position start, Position end)
{
    // Initialize weights in map starting from given position
    initializeFloodMap(start);
}

void PathFinder::initializeFloodMap(Position start)
{
    for(int x; x < rows.size(); x++) {
        for(int y; y < rows[x].cells.size(); y++) {
            
        }
    }

}

#endif // PATH_FINDER_LIB
