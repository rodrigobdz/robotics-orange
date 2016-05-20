#include "ros/ros.h"
#include "create_fundamentals/SensorPacket.h"
#include <basic_movements.cpp>
#include <wall.cpp>

class Ransac
{
  public:
    Ransac()
    {
        // Set up laser callback
        laserSubscriber = n.subscribe("scan_filtered", 1, &Ransac::laserCallback, this);

        ranges = *(new std::vector<float>(512));
    }

    std::vector<Wall*> getWalls();

  private:
    static const float POINT_COUNT_FOR_WALL = 150; // Matches that makes  line to wall
    static const float PI = 3.14159265;
    static const float LASER_COUNT = 512;
    static const float ITERATIONS = 1000; // Number of iterations from ransac algo.
    static const float ERROR = 0.01;      // Difference between line and points

    ros::NodeHandle n;
    ros::Subscriber laserSubscriber; // Laser

    std::vector<float> ranges;

    float calculateX(float angle, float distance);
    float calculateY(float angle, float distance);
    std::vector<int> getMatches(Wall wall);
    std::pair<float, float> getRandomXYCoords();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

void bubbleSort(std::vector<Wall*>& a);

/* *
 * Recognize walls with ransac
 *
 * Returns: two points that represent a wall {point1x, point1y, point2x, point2y}
 * */
std::vector<Wall*> Ransac::getWalls()
{
    // Get new sensor dates
    ros::spinOnce();

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

        Wall* currentWall = new Wall(pointA.first, pointA.second, pointB.first, pointB.second);

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

    // Sort the return std::vector
    // The smaller the angle the sooner in the array.
    if (walls.size() > 1) {
        bubbleSort(walls);
    }

    return walls;
}

void bubbleSort(std::vector<Wall*>& a)
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
std::vector<int> Ransac::getMatches(Wall wall)
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

/*
 * Calculates coordinates from a random points from the ranges.
 */
std::pair<float, float> Ransac::getRandomXYCoords()
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

void Ransac::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { ranges = msg->ranges; }
