#ifndef PATH_FINDER_LIB
#define PATH_FINDER_LIB

#include <constants.cpp>
#include <maze.cpp>

#include "orange_fundamentals/Grid.h"
#include "orange_fundamentals/Cell.h"
#include "orange_fundamentals/Row.h"

class PathFinder
{
    public:
        PathFinder(){}
    
        void find();

    private:
        bool DEBUG = true;
    		std::vector<Row> rows; // Map variable
};

void PathFinder::find()
{
		
}

#endif