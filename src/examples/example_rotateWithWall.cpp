#include <signal.h>
#include <basic_movements.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_rotateWithWall");
  signal(SIGINT, stopMotors);

  BasicMovements basic_movements;
  basic_movements.rotateLeft();
  basic_movements.stop();
  basic_movements.rotateRight();
  basic_movements.stop();
  basic_movements.rotateLeft();
}
