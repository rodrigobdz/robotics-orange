
// emacs-lisp:
// (setq c-basic-offset 4)

#ifndef MAZELIB
#define MAZELIB

#include <utility>
#include <algorithm>
#include <string>

/* macro ROTATE_? left|right: @param d is a direction which is mapped
   on the left|right hand side direction.
   macro ROTATEALL left: rotate left each element in @V. */
#define ROTATE_L(d) do {                  \
                     int di = (int) d;    \
                     di = (di+1)%4;       \
                     d = (directions) di; \
  } while (0)

#define ROTATE_R(d) do {                           \
                     int di = (int) d;             \
                     di = ((di-1)<0) ? 3 : (di-1); \
                     d = (directions) di;          \
  } while (0)

#define ROTATEALL(V) (                    \
    std::for_each((V).begin(), (V).end(), \
                  [](int &n) { ROTATE_L(n); }))

enum directions { RIGHT = 0, TOP, LEFT, BOTTOM };
char directions_lookup[] = { 'R', 'T', 'L', 'B' };

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

// FILTER

/* macro FILTERPATTERN: @param V is a vector of cell objects (structs) and
   p is a wallpattern. The macro than removes all cells which ain't having
   the same wallpattern. */
#define FILTERPATTERN(V,p) (                               \
    (V).erase(std::remove_if((V).begin(), (V).end(),       \
                             [&p](struct cell c) {         \
                               return !(c.pattern == (p)); \
                             }), (V).end()))


/* move: @param position is an integer array with length 3, which
   fields are 0: x coordinate in the maze, 1: y coordinate in the maze,
   2: orientation. The function changes the coordinates in place according
   to an move in the given direction. */
inline void move(int* position)
{
    switch(position[2]) {
    case RIGHT:  position[1] = position[1] + 1; break;
    case TOP:    position[0] = position[0] - 1; break;
    case LEFT:   position[1] = position[1] - 1; break;
    case BOTTOM: position[0] = position[0] + 1; break;
    default: ;
    }
}

#if 0 // experimental - for not working stuff and testing

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