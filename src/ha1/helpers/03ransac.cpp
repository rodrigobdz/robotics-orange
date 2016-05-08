#include <ctime>
#include <math.h>

// Number of matches in order for line 
// to be recognized as a wall
#define POINT_COUNT_FOR_WALL 170
#define PI 3.14159265
#define LASER_COUNT 512
#define ITERATIONS 1000
#define ERROR 0.02
#define LASER_OFFSET 0.13

// Variable to store laser values
std::vector<float> ranges(LASER_COUNT);

/*
 * m: Slope
 * ax, ay: First point
 * bx, by: Second point
 * px, py: Point
 *
 * Returns: Calculated distance to point in 
 *          cartesian coordinate system
 */
float distanceFromLineToPoint(float m, float ax, float ay, float bx, float by, float px, float py) 
{
  // Variables for Hesse normal form computation
  // from two-point form
  //    a*px + b*py - c = 0
  // --------------------------
  //      √( a² + b² )
  float a = ay-by;
  float b = bx-ax;
  float c = bx*ay-ax*by;
  float normalVector = sqrt( pow(a,2) +  pow(b,2) ); 

  return abs( (a*px+b*py-c)/normalVector );
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

/* *
 * Recognize a wall with ransac
 *
 * Returns: two points that represent a wall
 * */
void ransac()
{
  // Saves the best line from the comparison between
  // the current and the last one.
  float bestM = 0;
  float bestN = 0;
  float bestMatches = 0;
  float matches = 0;

  for (int i = 0; i < ITERATIONS; ++i) {
    // Number of points matching best line
    matches = 0;
    // Seed srand for rand use
    std::srand(std::time(NULL));
    // Take two random points from laser scanner
    float firstRandom = std::rand() % sizeof(ranges);
    float secondRandom = std::rand() % sizeof(ranges);
    
    // Distance to points
    //
    // index-1 to make last member of array also accessible
    float a = ranges[firstRandom-1];
    float b = ranges[secondRandom-1];

    // Ignore values if nan
    if(isnan(a) || isnan(b)) {
      continue;
    } 

    // Angles in radians from laser scanner first point to chosen points
    //
    // Ranges' indexes are upside down. ranges[0] has value of degree
    // 180 and ranges[LASER_COUNT-1] value of degree 0
    float alpha = PI/(LASER_COUNT) * (LASER_COUNT - firstRandom);
    float beta = PI/(LASER_COUNT) * (LASER_COUNT - secondRandom);

    // Points within own coordinate system
    float ax = calculateX(alpha, a);
    float ay = calculateY(alpha, a);
    float bx = calculateX(beta, b);
    float by = calculateY(beta, b);

    // Current line
    float m = firstRandom > secondRandom ? (ay - by) / (ax - bx) : (by - ay) / (bx - ax); // slope
    float n = by - (m * bx); // y-intercept

    // Iterate point cloud looking for points
    // close enough to current line
    for (int j = 0; j < LASER_COUNT; j++) {
      float p = ranges[j];

      // Ignore point if:
      // * nan
      // * is same as one of two chosen points for line
      if(isnan(p) || p == a || p == b) {
        continue;
      }

      // Calculate angle to point
      float gamma = PI/(LASER_COUNT) * (LASER_COUNT - j);

      float pointX = calculateX(ranges[j], gamma);
      float pointY = calculateY(ranges[j], gamma);

      float distance = distanceFromLineToPoint(m, ax, ay, bx, by, pointX, pointY);
      if (distance < ERROR) {
        matches++; 
      }
    }

    // Check if a better line fitting
    // point cloud was found
    if (matches > bestMatches) {
      bestMatches = matches;
      bestM = m;
      bestN = n;
    }
  }

  if(bestMatches >= POINT_COUNT_FOR_WALL ) {
    // Wall was found
  }
  
}