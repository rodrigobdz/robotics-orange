#include <signal.h>
#include <basic_movements.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_rotate");
  BasicMovements basic_movements;
  signal(SIGINT, stopMotors);

  // Rotate 90 degrees clockwise with a velocity of 7 rad/s
  basic_movements.rotateRight();
  basic_movements.rotateLeft();
  basic_movements.rotateRight();
  basic_movements.rotateLeft();
}


