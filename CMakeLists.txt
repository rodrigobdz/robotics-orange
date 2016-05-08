# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(orange_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(FILES
#)
#uncomment if you have defined services
#add_service_files(FILES
#)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
    CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(example_laser src/examples/example_laser.cpp)
target_link_libraries(example_laser ${catkin_LIBRARIES})
add_dependencies(example_laser create_fundamentals_generate_messages_cpp)

add_executable(example_moving src/examples/example_moving.cpp)
target_link_libraries(example_moving ${catkin_LIBRARIES})
add_dependencies(example_moving create_fundamentals_generate_messages_cpp)

add_executable(wanderer src/ha1/01wanderer.cpp)
target_link_libraries(wanderer ${catkin_LIBRARIES})
add_dependencies(wanderer create_fundamentals_generate_messages_cpp)

add_executable(perf_square src/ha1/02perf_square.cpp)
target_link_libraries(perf_square ${catkin_LIBRARIES})
add_dependencies(perf_square create_fundamentals_generate_messages_cpp)

# add_executable(perf_square_with_encoders src/ha1/02perf_square_with_encoders.cpp)
# target_link_libraries(perf_square_with_encoders ${catkin_LIBRARIES})
# add_dependencies(perf_square_with_encoders create_fundamentals_generate_messages_cpp)

add_executable(align src/ha1/03align.cpp)
target_link_libraries(align ${catkin_LIBRARIES})
add_dependencies(align create_fundamentals_generate_messages_cpp)