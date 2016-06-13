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
add_service_files(FILES ExecutePlan.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs create_fundamentals
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

# add_executable(example_driveWall src/examples/example_driveWall.cpp)
# target_link_libraries(example_driveWall ${catkin_LIBRARIES})
# add_dependencies(example_driveWall create_fundamentals_generate_messages_cpp)

# add_executable(test_laser src/tests/laser.cpp)
# target_link_libraries(test_laser ${catkin_LIBRARIES})
# add_dependencies(test_laser create_fundamentals_generate_messages_cpp)

add_executable(example_song src/examples/ros/example_song.cpp)
target_link_libraries(example_song ${catkin_LIBRARIES})
add_dependencies(example_song create_fundamentals_generate_messages_cpp)

                #################################
                #           HOMEWORKS           #
                #################################

# add_executable(global_localization src/ha3/01GlobalLocalization.cpp)
# target_link_libraries(global_localization ${catkin_LIBRARIES})
# add_dependencies(global_localization create_fundamentals_generate_messages_cpp)

add_executable(track_position_and_orientation src/ha3/02tracking_position_and_orientation.cpp)
target_link_libraries(track_position_and_orientation ${catkin_LIBRARIES})
add_dependencies(track_position_and_orientation create_fundamentals_generate_messages_cpp)

