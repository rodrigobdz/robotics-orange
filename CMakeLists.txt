# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(orange_fundamentals)
# Load catkin and all dependencies required for this package
find_package(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    create_fundamentals
    message_generation
    visualization_msgs
    std_msgs
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#uncomment if you have defined messages
add_message_files(
    FILES
    Cell.msg
    Grid.msg
    Pose.msg
    Row.msg
)

#uncomment if you have defined services
add_service_files(
    FILES
    ExecutePlan.srv
    MoveToPosition.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals
)

set(GCC_COVERAGE_COMPILE_FLAGS "--std=c++11")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

include_directories(include ${catkin_INCLUDE_DIRS} src/lib)

#############################################################################
#                               EXECUTABLES                                 #
#############################################################################

                    #################################
                    #            EXAMPLES           #
                    #################################

add_executable(example_playsong src/examples/example_playsong.cpp)
target_link_libraries(example_playsong ${catkin_LIBRARIES})
add_dependencies(example_playsong create_fundamentals_generate_messages_cpp)

add_executable(example_drive src/examples/example_drive.cpp)
target_link_libraries(example_drive ${catkin_LIBRARIES})
add_dependencies(example_drive create_fundamentals_generate_messages_cpp)

                    #################################
                    #           TOURNAMENT          #
                    #################################

add_executable(challenge_smooth src/tournament/challenge_smooth.cpp)
target_link_libraries(challenge_smooth ${catkin_LIBRARIES})
add_dependencies(challenge_smooth create_fundamentals_generate_messages_cpp)

add_executable(tournament src/tournament.cpp)
target_link_libraries(tournament ${catkin_LIBRARIES})
add_dependencies(tournament create_fundamentals_generate_messages_cpp)
