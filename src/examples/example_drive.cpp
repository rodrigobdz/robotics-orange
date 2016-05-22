#include <basic_movements.cpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_drive");

    BasicMovements basicMovements;
    // Drive one meter with a velocity of 6 rad/s
    basicMovements.rotate(1, 6);
}
