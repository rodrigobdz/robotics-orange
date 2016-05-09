#include <ctime>
#include <math.h>
#include <vector>
#include "constants.h"

// Number of matches in order for line
// to be recognized as a wall
#define POINT_COUNT_FOR_WALL 
#define PI 3.14159265
#define LASER_COUNT 512
#define ITERATIONS 5000
#define ERROR 0.01

// Variable to store laser values
std::vector<float> ranges(LASER_COUNT);

/*
 * x1, y1: First point
 * x2, y2: Second point
 * px, py: Point
 *
 * Returns: Calculated distance to point in
 *          cartesian coordinate system
 */
float distanceFromLineToPoint(float x1, float y1, float x2, float y2, float px, float py)
{
  // Variables for Hesse normal form computation
  // from two-point form
  //    a*px + b*py - c = 0
  // --------------------------
  //      √( a² + b² )
  float a = y2-y1;
  float b = x2-x1;
  float c = x2*y1-x1*y2;
  float normalVector = sqrt(pow(a,2) + pow(b,2));

  return fabs((a*px+b*py-c)/normalVector);
}

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: x coordinate in robot coordinate system
**/
float calculateX(float angle, float distance)
{
  return distance * cos(angle);
}

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: y coordinate in robot coordinate system
**/
float calculateY(float angle, float distance)
{
  return distance * sin(angle);
}

/*
 * Calculates coordinates from a random points from the ranges.
 */
std::pair<float, float> getRandomXYCoords()
{
    float randomNumber = std::rand() % ranges.size();

    // Distance to points
    // index-1 to make last member of array also accessible
    float a = ranges[randomNumber-1];

    // Ignore values if nan
    if(isnan(a)) {
      return std::pair<float,float>(NAN, NAN);
    }

    // Angles in radians from laser scanner first point to chosen points
    float angle = PI/(LASER_COUNT) * randomNumber;

    // Points within own coordinate system
    return std::pair<float,float>(calculateX(angle, a), calculateY(angle, a));
}

/*
 * Calculates the matches from given line to points from ranges.
 *
 * Return: Number of matches
 */
int getMatches(std::vector<float> line){

  int matches = 0;

  // Iterate point cloud looking for points
  // close enough to current line
  for (int j = 0; j < LASER_COUNT; j++) {
    float distanceFromRobotToPoint = ranges[j];

    // Ignore point if:
    // * nan
    // * is same as one of two chosen points for line
    if(isnan(distanceFromRobotToPoint)) {
      continue;
    }

    // Calculate angle to point
    float angle = PI/(LASER_COUNT) * j;

    float pointX = calculateX(ranges[j], angle);
    float pointY = calculateY(ranges[j], angle);

    if (distanceFromLineToPoint(line[0], line[1], line[2], line[3], pointX, pointY) < ERROR) {
      matches++;
    }
  }
  return matches;
}

/* *
 * Recognize a wall with ransac
 *
 * Returns: two points that represent a wall {point1x, point1y, point2x, point2y}
 * */
std::vector<float> ransac()
{
  // Variables
  std::vector<float> bestLine(4);
  int bestMatches = 0;

  // Seed for Random Generator
  std::srand(std::time(NULL));

  for (int i = 0; i < ITERATIONS; ++i) {

    // Get coordiantes of two random selected points
    std::pair<float, float> pointA = getRandomXYCoords();
    std::pair<float, float> pointB = getRandomXYCoords();

    std::vector<float> currentLine(4);
    currentLine[0] = pointA.first;
    currentLine[1] = pointA.second;
    currentLine[2] = pointB.first;
    currentLine[3] = pointB.second;

    float matches = getMatches(currentLine);

    // Check if currentLine better line fitting
    // point cloud was found
    if (matches > bestMatches) {
      bestMatches = matches;
      bestLine = currentLine;
    }
  }

  if(bestMatches >= POINT_COUNT_FOR_WALL ) {
    // Wall was found
    ROS_INFO("Matches: %i", bestMatches);
    return bestLine;
  }

  std::vector<float> error(4);
  error[0] = NAN;
  error[1] = NAN;
  error[2] = NAN;
  error[3] = NAN;

  return error;
}
