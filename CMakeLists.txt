# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(orange_fundamentals)
# Load catkin and all dependencies required for this package



find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    rospy
                    sensor_msgs
                    create_fundamentals
                    message_generation
                    )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(FILES
#)
#uncomment if you have defined services



add_service_files(FILES ExecutePlan.srv)



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

include_directories(include ${catkin_INCLUDE_DIRS} ${YOUR_DIRECTORY}/src/lib)

# add_executable(example_drive src/examples/example_drive.cpp)
# target_link_libraries(example_drive ${catkin_LIBRARIES})
# add_dependencies(example_drive create_fundamentals_generate_messages_cpp)

# add_executable(example_rotate src/examples/example_rotate.cpp)
# target_link_libraries(example_rotate ${catkin_LIBRARIES})
# add_dependencies(example_rotate create_fundamentals_generate_messages_cpp)

add_executable(03align src/ha1/03align.cpp)
target_link_libraries(03align ${catkin_LIBRARIES})
add_dependencies(03align create_fundamentals_generate_messages_cpp)

add_executable(execute_plan src/ha2/01moving_through_the_grid.cpp)
target_link_libraries(execute_plan ${catkin_LIBRARIES})
add_dependencies(execute_plan create_fundamentals_generate_messages_cpp)
