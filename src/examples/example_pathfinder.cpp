#include <signal.h>
#include <path_finder.cpp>
#include <basic_movements.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_align");
  signal(SIGINT, stopMotors);
  
  // PathFinder pathFinder;
  // pathFinder.find();

  return 0;
}