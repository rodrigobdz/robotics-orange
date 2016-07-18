#include <signal.h>
#include <plan.cpp>
#include <maze.cpp>
#include <position.cpp>
#include <path_finder.cpp>
#include <play_song.cpp>
#include <basic_movements.cpp>

static const bool DEBUG = true;

void stopMotors(int signal) {
    BasicMovements basic_movements;
    basic_movements.stop();
    ros::shutdown();
}

Position findNearestPosition(Position currentPosition, std::vector<Position> positions)
{
    PathFinder path_finder;
    Position nearestPosition;
    int shortestLength = 100000;

    for (int i = 0; i < positions.size(); i++) {
        std::vector<int> plan_instructions = path_finder.find(currentPosition, positions[i]);
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
        if (positions[i].getXCoordinate() == toDelete.getXCoordinate() &&
            positions[i].getYCoordinate() == toDelete.getYCoordinate()) {
        } else {
            newPositions.push_back(positions[i]);
        }
    }
    return newPositions;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tournament");
    signal(SIGINT, stopMotors);

    Plan plan;
    Maze maze;
    PathFinder path_finder;
    PlaySongLib play_song;

    Position gold1{0, 3, -1};
    Position gold2{2, 4, -1};
    Position gold3{2, 0, -1};
    Position gold4{3, 2, -1};
    Position gold5{3, 5, -1};
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
        shortestPath = path_finder.find(currentPosition, nearestPosition);

        ROS_INFO("CurrentPosition : %i, %i, %i", currentPosition.getXCoordinate(), currentPosition.getYCoordinate(), currentPosition.getDirection());
        ROS_INFO("Remaining plans: %lu", goldPositions.size());
        ROS_INFO("Drive to: %i, %i, %i",  nearestPosition.getXCoordinate(), nearestPosition.getYCoordinate(), nearestPosition.getDirection());
        printf("Drive plan: ");

        for(int i = 0; i < shortestPath.size(); i++){
            printf("%i ", shortestPath[i]);
        }
        printf("\n");
        if (!plan.executeSmooth(shortestPath, currentPosition.getDirection())) {
            if(DEBUG) {
                ROS_INFO("Execute plan smooth failed");
            }
            maze.relocalize();
            continue;
        }

        goldPositions = deletePosition(goldPositions, nearestPosition);

        maze.updatePositionOnMap(shortestPath);
        ROS_INFO("starWarsShort");
        play_song.starWarsShort();
    }

    currentPosition = maze.getPosition();
    nearestPosition = findNearestPosition(currentPosition, pickupPositions);
    shortestPath = path_finder.find(currentPosition, nearestPosition);

    plan.execute(shortestPath, currentPosition.getDirection());

	return 0;
}
