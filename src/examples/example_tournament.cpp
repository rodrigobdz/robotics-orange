#include <signal.h>
#include <path_finder.cpp>
#include <basic_movements.cpp>
#include <plan.cpp>

void stopMotors(int signal)
{
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_tournament");
    signal(SIGINT, stopMotors);

    // PathFinder path_finder;

    // std::vector<std::vector<int>> map;
    // basic_movements.move(1,0);
    //
    Plan plan;

    std::vector<int> planOrders = {0,0};
    planOrders = plan.makeSmoothPlan(planOrders);

    ROS_INFO("Size of planOrders %lu", planOrders.size());
    for (int i = 0; i < planOrders.size(); ++i) {
        ROS_INFO("PlanOrders %d = %d", i, planOrders[i]);
    }

    // path_finder.initializeWeightedMap({0,0,0});
    // map = path_finder.weightedMap;
    // ROS_INFO("Size %lu %lu", map.size(), map[0].size());

    // for (int i = 0; i < 5; ++i)
    // {
    //  ROS_INFO("%i %i %i %i %i %i %i", map[0][i], map[1][i], map[2][i], map[3][i], map[4][i], map[5][i], map[6][i]);
    // }

    // // std::vector<int> vInt{0,1,2};
    // // ROS_INFO("Check %d != %d ? %d", std::find(vInt.begin(), vInt.end(), 3), vInt.end(), std::find(vInt.begin(),
    // vInt.end(), 3) !=  vInt.end());

    // path_finder.setDistancesInWeightedMap({0,0,1});
    // map = path_finder.weightedMap;

    // for (int i = 0; i < 5; ++i)
    // {
    //  ROS_INFO("%i %i %i %i %i %i %i", map[0][i], map[1][i], map[2][i], map[3][i], map[4][i], map[5][i], map[6][i]);
    // }

    // std::vector<int> plan = path_finder.findShortestPath({0,0,0}, {5,3,0});

    // std::vector<int> plan = path_finder.find({0,4,0}, {6,0,0});
    // // plan.insert(plan.begin(), 3);

    return 0;
}

