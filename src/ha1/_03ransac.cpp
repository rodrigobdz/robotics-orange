#include <ctime>
#include <math.h>

#define PI 3.14159265
#define LASER_COUNT 512
#define ERROR 0.02

std::vector<float> ranges(LASER_COUNT);

float distanceFromLineToPoint(float m, float n, float x, float y) {
  return 0;
}

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: x coordinate in robot coordinate system 
**/
float calculateX(float angle, float distance) 
{
  return angle <= PI/2 ? cos(angle) * distance : sin(angle - PI/2) * distance;
}

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: y coordinate in robot coordinate system 
**/
float calculateY(float angle, float distance) 
{
  return angle <= PI/2 ? sin(angle) * distance : cos(angle - PI/2) * distance;
}

bool recWall() 
{
  return false;
}

/* *
 * Recognize a wall with ransac
 * Returns two points of the wall
 * */
void ransac()
{
  int iterations = 1000;
  // Saves the best line from the comparison between
  // the current line and the last one.
  float bestM;
  float bestN;
  float bestMatches = 0;

  for (int i = 0; i < iterations; ++i) {
    // Number of points matching best line
    float matches = 0;
    // Seed srand for rand use
    std::srand(std::time(NULL));
    // Take two random points from laser scanner
    float firstRandom = std::rand() % sizeof(ranges);
    float secondRandom = std::rand() % sizeof(ranges);
    // Distance to points
    float a = ranges[firstRandom];
    float b = ranges[secondRandom];

    // Angles from laser scanner first point to chosen points
    float alpha = PI/(LASER_COUNT) * firstRandom;
    float beta = PI/(LASER_COUNT) * secondRandom;

    // Points within own coordinate system
    float ax = calculateX(alpha, a);
    float ay = calculateY(alpha, a);
    float bx = calculateX(beta, b);
    float by = calculateY(beta, b);

    // Current line
    float m = firstRandom > secondRandom ? (ay - by) / (ax - bx) : (by - ay) / (bx - ax); // slope
    float n = by - (m * bx); // y-intercept

    for (int j = 0; j < LASER_COUNT; j++) {
      // Calculate angle to point
      float gamma = PI/(LASER_COUNT) * j;

      float pointX = calculateX(ranges[j], gamma);
      float pointY = calculateY(ranges[j], gamma);

      float distance = distanceFromLineToPoint(m, n, pointX, pointY);
      if (distance > ERROR) matches++; 
    }

    // Check if a line that fits the point cloud 
    // better was found
    if (matches > bestMatches) {
      bestMatches = matches;
      bestM = m;
      bestN = n;
    }
  }
}