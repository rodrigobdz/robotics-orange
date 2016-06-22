#ifndef CONSTANTS_LIB
#define CONSTANTS_LIB

// Import pi constant (M_PI)
#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

// Distances are given in meters
static const float DISTANCE_LASER_TO_ROBOT_CENTER = 0.125;
static const float CELL_CENTER                    = 0.39;
static const float SAFETY_DISTANCE                = 0.30; // Minimum distance to keep when driving
static const float CELL_LENGTH                    = 0.80;
static const float LASER_MAX_REACH                = 1;

static const float PI                             = M_PI;
static const int MAX_INT                          = std::numeric_limits<int>::max();
static const float LASER_COUNT                    = 512;
static const float ROB_DIAMETER                   = 0.34;
static const float ROB_BASE                       = 0.258;
static const float RAD_RADIUS                     = 0.0325;
static const float ONE_METER_IN_RAD               = 1 / RAD_RADIUS;
static const float NINETY_DEGREES_IN_RAD          = 30.798 * 0.196349; // 2pir^2 / 4, r = 0.26/2
static constexpr float DEFAULT_SPEED              = 5;

static const int LOOP_RATE                        = 16; // Used for loop rate

// Robot position
static const int RIGHT                            = 0;
static const int UP                               = 1;
static const int LEFT                             = 2;
static const int DOWN                             = 3;

#endif // CONSTANTS_LIB
