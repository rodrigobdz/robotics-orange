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
#                          EXECUTABLES                                      #
#############################################################################

                #################################
                #            EXAMPLES           #
                #################################

add_executable(example_driveWall src/examples/example_driveWall.cpp)
target_link_libraries(example_driveWall ${catkin_LIBRARIES})
add_dependencies(example_driveWall create_fundamentals_generate_messages_cpp)

add_executable(example_ransac src/examples/example_ransac.cpp)
target_link_libraries(example_ransac ${catkin_LIBRARIES})
add_dependencies(example_ransac create_fundamentals_generate_messages_cpp)

# add_executable(test_laser src/tests/laser.cpp)
# target_link_libraries(test_laser ${catkin_LIBRARIES})
# add_dependencies(test_laser create_fundamentals_generate_messages_cpp)

# add_executable(ransac_tb src/tests/ransac_tb.cpp)
# target_link_libraries(ransac_tb ${catkin_LIBRARIES})
# add_dependencies(ransac_tb create_fundamentals_generate_messages_cpp)

# add_executable(env_tb src/tests/env_tb.cpp)
# target_link_libraries(env_tb ${catkin_LIBRARIES})
# add_dependencies(env_tb create_fundamentals_generate_messages_cpp)

                #################################
                #           HOMEWORKS           #
                #################################

# add_executable(03align src/ha1/03align.cpp)
# target_link_libraries(03align ${catkin_LIBRARIES})
# add_dependencies(03align create_fundamentals_generate_messages_cpp)

# add_executable(execute_plan src/ha2/01moving_through_the_grid.cpp)
# target_link_libraries(execute_plan ${catkin_LIBRARIES})
# add_dependencies(execute_plan create_fundamentals_generate_messages_cpp)

# add_executable(execute_plan_smooth src/ha2/02moving_fast_and_smooth.cpp)
# target_link_libraries(execute_plan_smooth ${catkin_LIBRARIES})
# add_dependencies(execute_plan_smooth create_fundamentals_generate_messages_cpp)

add_executable(global_localization src/ha3/01GlobalLocalization.cpp)
target_link_libraries(global_localization ${catkin_LIBRARIES})
add_dependencies(global_localization create_fundamentals_generate_messages_cpp)

