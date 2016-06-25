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

Position findNearestPosition(Position currentPosition, std::vector<Position> positions)
{
	PathFinder pathFinder;
    Position nearestPosition;
    int shortestLength = 100000;

    for (int i = 0; i < positions.size(); i++) {
        std::vector<int> plan_instructions = pathFinder.find(currentPosition, positions[i]);
        if (shortestLength > plan_instructions.size()) {
            nearestPosition = positions[i];
            shortestLength = plan_instructions.size();
        }
    }

    return nearestPosition;
}

std::vector<Position> deletePosition(std::vector<Position> positions, Position toDelete)
{
    std::vector<Position> newPositions;
    for (int i = 0; i < positions.size(); i++) {
        if (positions[i].getXCoordinate() != toDelete.getXCoordinate() &&
            positions[i].getYCoordinate() != toDelete.getYCoordinate()) {
            newPositions.push_back(positions[i]);
        }
    }
    return newPositions;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "example_plan");
    signal(SIGINT, stopMotors);

    Plan plan;
    Maze maze;
    PathFinder pathFinder;

    Position gold1{0, 3, -1};
    Position gold2{2, 4, -1};
    Position gold3{2, 0, -1};
    Position gold4{3, 2, -1};
    Position gold5{3, 3, -1};
    std::vector<Position> goldPositions{gold1, gold2, gold3, gold4, gold5};

    Position pickup1{1, 2, -1};
    Position pickup2{4, 0, -1};
    std::vector<Position> pickupPositions{pickup1, pickup2};

    Position currentPosition;
    Position nearestPosition;
    std::vector<int> shortestPath;

    while(goldPositions.size()  > 0){
        currentPosition = maze.getPosition();
        nearestPosition = findNearestPosition(currentPosition, goldPositions);
        shortestPath = pathFinder.find(currentPosition, nearestPosition);

        plan.execute(shortestPath, currentPosition.getDirection());
        goldPositions = deletePosition(goldPositions, nearestPosition);

        // Update current position

    }

    currentPosition = maze.getPosition();
    nearestPosition = findNearestPosition(currentPosition, pickupPositions);
    shortestPath = pathFinder.find(currentPosition, nearestPosition);

    plan.execute(shortestPath, currentPosition.getDirection());

	return 0;
}
