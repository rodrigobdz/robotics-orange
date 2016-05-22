#ifndef CONSTANTS_LIB
#define CONSTANTS_LIB

// Import pi constant (M_PI)
#define _USE_MATH_DEFINES
#include <cmath>

// Distances are given in meters
static const float DISTANCE_LASER_TO_ROBOT_CENTER = 0.125;
static const float CELL_CENTER                    = 0.39;		
static const float SAFETY_DIS                     = 0.15; // Minimum distance to keep when driving
static const float CELL_LENGTH                    = 0.80;

static const float PI                             = M_PI;
static const float LASER_COUNT                    = 512;
static const float ONE_METER_IN_RAD               = 30.798;
static const float NINETY_DEGREES_IN_RAD          = 30.798 * 0.196349; // 2pir^2 / 4, r = 0.26/2

#endif // CONSTANTS_LIB