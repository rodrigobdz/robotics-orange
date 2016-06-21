#include <basic_movements.cpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_drive");

    BasicMovements basicMovements;
    basicMovements.drive(-0.2, 5);
}
