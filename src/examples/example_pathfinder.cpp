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

	std::vector<Position> neighbours = path_finder.getNeighbours({6,4,1});

	for (int i = 0; i < neighbours.size(); ++i)
	{
		neighbours[i].print();
	}
	// std::vector<int> vInt{0,1,2};
	// ROS_INFO("Check %d != %d ? %d", std::find(vInt.begin(), vInt.end(), 3), vInt.end(), std::find(vInt.begin(), vInt.end(), 3) !=  vInt.end());

	// path_finder.setDistancesInWeightedMap({0,0,1}, {2,4,1});

	// for (int i = 0; i < 5; ++i) 
	// {
	// 	ROS_INFO("%i %i %i %i %i %i %i", map[0][i], map[1][i], map[2][i], map[3][i], map[4][i], map[5][i], map[6][i]);
	// }

  	return 0;
}