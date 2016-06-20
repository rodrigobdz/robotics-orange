#include <signal.h>
#include <plan.cpp>
#include <basic_movements.cpp>

void mySigintHandler(int signal) {
    BasicMovements basicMovements;
    basicMovements.stop();
    ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "example_plan");

	std::vector<int> plan_instructions = {UP, RIGHT, DOWN, RIGHT, RIGHT, UP, RIGHT, UP, LEFT};

	Plan plan;
	plan.execute(plan_instructions);

	signal(SIGINT, mySigintHandler);

	return 0;
}