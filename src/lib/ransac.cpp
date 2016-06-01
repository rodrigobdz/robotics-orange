#ifndef RANSAC_LIB
#define RANSAC_LIB

#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include "sensor_msgs/LaserScan.h"
#include <wall.cpp>
#include <constants.cpp>

class Ransac
{
  public:
    Ransac()
    {
        // Set up laser callback
        laserSubscriber = n.subscribe("scan_filtered", 1, &Ransac::laserCallback, this);

        ranges          = *(new std::vector<float>(LASER_COUNT));
        srand(time(NULL));
    }

    std::vector<Wall*> getWalls();

  private:
    ///////////////////
    //   Variables   //
    ///////////////////
    static const float TRESHOLD   = 100;  // Matches that makes  line to wall
    static const float ITERATIONS = 400; // Number of iterations from ransac algo.
    static const float ERROR      = 0.02; // Difference between line and points

    ros::NodeHandle n;
    ros::Subscriber laserSubscriber;
    std::vector<float> ranges;

    ///////////////////
    //   Functions   //
    ///////////////////
    float calculateX(float angle, float distance);
    float calculateY(float angle, float distance);
    std::vector<int> getMatches(float wallX1, float wallY1, float wallX2, float wallY2);
    std::pair<float, float> getRandomXYCoords();

    void bubbleSort(std::vector<Wall*>& a);

    void initialiseLaser();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
}; /* *
 * Recognize walls with ransac
 *
 * Returns: two points that represent a wall {point1x, point1y, point2x, point2y}
 * */
std::vector<Wall*> Ransac::getWalls()
{
    std::vector<Wall*> walls;

    // Get new sensor dates
    initialiseLaser();

    // We can maximal acknowledge 3 walls
    for(int j = 0; j < 2; j++){

        // Variables
        std::vector<int> bestMatches;
        Wall *bestWall;

        for (int i = 0; i < ITERATIONS; ++i) {
            // Get coordiantes of two random selected points
            std::pair<float, float> pointA = getRandomXYCoords();
            std::pair<float, float> pointB = getRandomXYCoords();

            if (isnan(pointA.first) || isnan(pointB.first)) {
                continue;
            }

            Wall* currentWall = new Wall(pointA.first, pointA.second, pointB.first, pointB.second);

            std::vector<int> currentMatches = getMatches(pointA.first, pointA.second, pointB.first, pointB.second);

            if(currentMatches.size() > bestMatches.size()){
                bestMatches = currentMatches;
                bestWall = currentWall;
            }
        }

        // Check if currentLine better line fitting
        // point cloud was found
        if (bestMatches.size() > TRESHOLD) {
            walls.push_back(bestWall);

            // Set found matches to nan
            // Better than to delete elements,
            // because we need to calculate the angle
            for (int j = 0; j < bestMatches.size(); j++) {
                ranges[bestMatches[j]] = NAN;
            }
        }
    }

    // Sort the return std::vector
    // The smaller the angle the sooner in the array.
    if (walls.size() > 1) {
        bubbleSort(walls);
    }

    return walls;
}

void Ransac::bubbleSort(std::vector<Wall*>& a)
{
    bool swapp = true;
    Wall* tmp;
    while (swapp) {
        swapp = false;
        for (int i = 0; i < a.size() - 1; i++) {
            if (a[i]->getAngle() > a[i + 1]->getAngle()) {
                tmp = a[i];
                a[i] = a[i + 1];
                a[i + 1] = tmp;
                swapp = true;
            }
        }
    }
}

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: x coordinate in robot coordinate system
 **/
float Ransac::calculateX(float angle, float distance) { return distance * cos(angle); }

/*
 * angle: from laser scanner first point to some point
 * distance: from laser scanner to point
 *
 * Returns: y coordinate in robot coordinate system
 **/
float Ransac::calculateY(float angle, float distance) { return distance * sin(angle); }

/*
 * Calculates the matches from given line to points from ranges.
 *
 * Return: Number of matches
 */
std::vector<int> Ransac::getMatches(float wallX1, float wallY1, float wallX2, float wallY2)
{
    std::vector<int> matches;
    float angle;
    float distanceFromRobotToPoint;

    float a = wallY1 - wallY2;
    float b = wallX2 - wallX1;
    float c = wallX2 * wallY1 - wallX1 * wallY2;
    float normalVector = sqrt(pow(a, 2) + pow(b, 2));

    // Iterate point cloud looking for points
    // close enough to current wall
    for (int j = 0; j < LASER_COUNT; j++) {

        // Calculate angle to point
        angle = j * (PI / LASER_COUNT);
        distanceFromRobotToPoint = ranges[j];

        if (isnan(distanceFromRobotToPoint)) {
            continue;
        }

        // Coords to point
        float px = calculateX(angle, ranges[j]);
        float py = calculateY(angle, ranges[j]);

        // Calculate distance from line to point
        float distance = fabs((a * px + b * py - c) / normalVector);

        if (distance < ERROR) {
            matches.push_back(j);
        }
    }
    return matches;
}

/*
 * Calculates coordinates from a random points from the ranges.
 */
std::pair<float, float> Ransac::getRandomXYCoords()
{
    int randomNumber = rand() % ranges.size() - 1;

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

/********************** HELPERS *****************************/

void Ransac::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { ranges = msg->ranges; }

void Ransac::initialiseLaser()
{
    ranges[0] = -1;
    ros::spinOnce();
    while (ranges[0] == -1) {
        // Get laser data before driving to recognize obstacles beforehand
        ros::spinOnce();
    }
}

#endif
