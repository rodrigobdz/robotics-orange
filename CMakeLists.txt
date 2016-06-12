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

include_directories(include ${catkin_INCLUDE_DIRS} src/lib)

#############################################################################
#                               EXECUTABLES                                 #
#############################################################################

#################################
#            EXAMPLES           #
#################################

add_executable(example_driveWall src/examples/example_driveWall.cpp)
target_link_libraries(example_driveWall ${catkin_LIBRARIES})
add_dependencies(example_driveWall create_fundamentals_generate_messages_cpp)

# add_executable(example_ransac src/examples/example_ransac.cpp)
# target_link_libraries(example_ransac ${catkin_LIBRARIES})
# add_dependencies(example_ransac create_fundamentals_generate_messages_cpp)
#
# add_executable(example_marker src/examples/example_marker.cpp)
# target_link_libraries(example_marker ${catkin_LIBRARIES})
# add_dependencies(example_marker create_fundamentals_generate_messages_cpp)

#################################
#           HOMEWORKS           #
#################################

# add_executable(execute_plan_smooth src/ha2/02moving_fast_and_smooth.cpp)
# target_link_libraries(execute_plan_smooth ${catkin_LIBRARIES})
# add_dependencies(execute_plan_smooth create_fundamentals_generate_messages_cpp)
