#include <signal.h>
#include <plan.cpp>
#include <basic_movements.cpp>

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_plan");
	signal(SIGINT, stopMotors);

	std::vector<int> plan_instructions = {UP, RIGHT, DOWN, RIGHT, RIGHT, UP, RIGHT, UP, LEFT};

	Plan plan;
	plan.execute(plan_instructions);

	return 0;
}