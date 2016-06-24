#include <signal.h>
#include <plan.cpp>
#include <maze.cpp>
#include <position.cpp>
#include <path_finder.cpp>
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

	Maze maze;
	PathFinder pathFinder;
	Position currentPosition = maze.getPosition();
	Position goalPosition{6,0,0};
	std::vector<int> plan_instructions = pathFinder.find(currentPosition, goalPosition);

	Plan plan;
	plan.execute(plan_instructions, currentPosition.getDirection());

	return 0;
}
