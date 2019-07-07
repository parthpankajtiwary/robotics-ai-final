cmake_minimum_required(VERSION 2.8.3)
project(alice_headcontroller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  alice_msgs
  control_msgs
  std_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/alice_headcontroller_node.cpp
                                    src/get_position.cpp
                                    src/move_head.cpp
                                    src/init.cpp
                                    src/cmd_vel.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
