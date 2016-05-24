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
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#uncomment if you have defined messages
#add_message_files(FILES
#)
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
#                          EXECUTABLES
#############################################################################

# add_executable(example_drive src/examples/example_drive.cpp)
# target_link_libraries(example_drive ${catkin_LIBRARIES})
# add_dependencies(example_drive create_fundamentals_generate_messages_cpp)

add_executable(example_rotate src/examples/example_rotate.cpp)
target_link_libraries(example_rotate ${catkin_LIBRARIES})
add_dependencies(example_rotate create_fundamentals_generate_messages_cpp)

# add_executable(03align src/ha1/03align.cpp)
# target_link_libraries(03align ${catkin_LIBRARIES})
# add_dependencies(03align create_fundamentals_generate_messages_cpp)

add_executable(execute_plan src/ha2/01moving_through_the_grid.cpp)
target_link_libraries(execute_plan ${catkin_LIBRARIES})
add_dependencies(execute_plan create_fundamentals_generate_messages_cpp)

# add_executable(ransac_tb src/tests/ransac_tb.cpp)
# target_link_libraries(ransac_tb ${catkin_LIBRARIES})
# add_dependencies(ransac_tb create_fundamentals_generate_messages_cpp)

# add_executable(env_tb src/tests/env_tb.cpp)
# target_link_libraries(env_tb ${catkin_LIBRARIES})
# add_dependencies(env_tb create_fundamentals_generate_messages_cpp)
