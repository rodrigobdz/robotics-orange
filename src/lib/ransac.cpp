#include <ctime>
#include <math.h>
#include <vector>
#include "wall.cpp"
#include "ranges.h"

// Number of matches in order for line
// to be recognized as a wall
#define POINT_COUNT_FOR_WALL 200
#define PI 3.14159265
#define LASER_COUNT 512
#define ITERATIONS 1000
#define ERROR 0.01

// Variable to store laser values
std::vector<float> ranges(LASER_COUNT);

namespace Ransac
{
/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: x coordinate in robot coordinate system
 **/
float calculateX(float angle, float distance) { return distance * cos(angle); }

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: y coordinate in robot coordinate system
 **/
float calculateY(float angle, float distance) { return distance * sin(angle); }

/*
 * Calculates coordinates from a random points from the ranges.
 */
std::pair<float, float> getRandomXYCoords()
{
    int randomNumber = std::rand() % ranges.size() - 1;

    // Distance to points
    // index-1 to make last member of array also accessible
    float a = ranges[randomNumber];

    // Ignore values if nan
    if (isnan(a)) {
        return std::pair<float, float>(NAN, NAN);
    }

    // Angles in radians from laser scanner first point to chosen points
    float angle = (PI / LASER_COUNT) * randomNumber;

    // Points within own coordinate system
    return std::pair<float, float>(calculateX(angle, a), calculateY(angle, a));
}

/*
 * Calculates the matches from given line to points from ranges.
 *
 * Return: Number of matches
 */
std::vector<int> getMatches(Wall wall)
{
    std::vector<int> matches;

    // Iterate point cloud looking for points
    // close enough to current wall
    for (int j = 0; j < LASER_COUNT; j++) {
        float distanceFromRobotToPoint = ranges[j];

        // Ignore point if:
        // * nan
        // * is same as one of two chosen points for wall
        if (isnan(distanceFromRobotToPoint)) {
            continue;
        }

        // Calculate angle to point
        float angle = (PI / LASER_COUNT) * j;

        float pointX = calculateX(angle, ranges[j]);
        float pointY = calculateY(angle, ranges[j]);

        if (wall.getDistance(pointX, pointY) < ERROR) {
            matches.push_back(j);
        }
    }
    return matches;
}

/* *
 * Recognize walls with ransac
 *
 * Returns: two points that represent a wall {point1x, point1y, point2x, point2y}
 * */
std::vector<Wall*> ransac()
{
    // Variables
    std::vector<Wall*> walls;

    // Seed for Random Generator
    std::srand(std::time(NULL));

    for (int i = 0; i < ITERATIONS; ++i) {
        // Get coordiantes of two random selected points
        std::pair<float, float> pointA = getRandomXYCoords();
        std::pair<float, float> pointB = getRandomXYCoords();

        if (isnan(pointA.first)) {
            continue;
        }

        Wall *currentWall = new Wall(pointA.first, pointA.second, pointB.first, pointB.second);

        std::vector<int> matches = getMatches(*currentWall);
        int matchSize = matches.size();

        // Check if currentLine better line fitting
        // point cloud was found
        if (matchSize > POINT_COUNT_FOR_WALL) {
            walls.push_back(currentWall);
            // Set found matches to nan
            // Better than to delete elements,
            // because we need to calculate the angle
            for (int j = 0; j < matchSize; j++) {
                ranges[matches[j]] = NAN;
            }
        }
    }

    return walls;
}
}
