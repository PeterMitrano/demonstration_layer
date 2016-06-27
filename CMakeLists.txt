cmake_minimum_required(VERSION 2.8.3)
project(demonstration_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES demonstration_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(demonstration_layer src/demonstration_layer.cpp)
target_link_libraries(demonstration_layer
   ${catkin_LIBRARIES}
)
