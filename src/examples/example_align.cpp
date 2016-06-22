#include <signal.h>
#include <environment.cpp>
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
  
  Env env;
  env.align();

  return 0;
}
