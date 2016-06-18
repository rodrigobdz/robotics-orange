#include <environment.cpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_align");
  Env env;
  env.align();

  return 0;
}
