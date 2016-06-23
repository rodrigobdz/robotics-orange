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

 	PathFinder path_finder;
  
 	std::vector<std::vector<int>> map;

	path_finder.initializeWeightedMap({0,2,0});
	map = path_finder.weightedMap;
	ROS_INFO("Test");
	ROS_INFO("Size %lu %lu", map.size(), map[0].size());
	
	for (int i = 0; i < 5; ++i) 
	{
		ROS_INFO("%i %i %i %i %i %i %i", map[0][i], map[1][i], map[2][i], map[3][i], map[4][i], map[5][i], map[6][i]);
	}

  	return 0;
}