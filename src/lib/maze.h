#ifndef MAZELIB
#define MAZELIB

#include <utility>
#include <algorithm>
#include <string>

// there are four posibilities how walls orientation could be
//  - 1 wall  -> just a wall
//  - 2 walls -> can eather be a corner (|_) or a path (| |)
//  - 3 walls -> is just a death end (|_|)
enum wallpattern { failure = -1, corner, end, path, wall };

// lookup table for tostring functions
std::string wp_lookup[] = { "corner", "end", "path", "wall" };

// representation of a cell
// contains coordinates in the matrix like @maze [(0,0) is
// the upper left cell], some information about the wall
// orientation and an integer array representing the walls
struct cell {
    std::pair<int,int> coordinates;
    std::vector<int> walls;
    wallpattern pattern;
};

// macro INDEX: calculate the index out of a tuple of coordinates
//   needed because we are faking a matrix with an array
//   used to access a cell in the maze array by coordinates
#define INDEX(p) (DIMENSION*((p).first)+((p).second))
#define DIMENSION 3

cell maze[DIMENSION*DIMENSION];

#if 0 // experimental - for not working stuff and testing

// FILTER
/* use this filter functions to get cells which match a wallpattern */
bool filterWallIsCorner(struct cell c) { return !(c.pattern == corner); }
bool filterWallIsPath(struct cell c)   { return !(c.pattern == path); }
bool filterWallIsEnd(struct cell c)    { return !(c.pattern == end); }
bool filterWallIsWall(struct cell c)   { return !(c.pattern == wall); }

#define GETCORNERS() do { \
        std::vector<cell> filteredMaze (DIMENSION*DIMENSION); \
        std::copy(maze, maze + (DIMENSION*DIMENSION), filteredMaze.begin()); \
        std::remove_if(filteredMaze.begin(), filteredMaze.end(), filterWallIsCorner), filteredMaze.end(); \
    } while (0)

#endif // experimental corner
#endif // MAZELIB
