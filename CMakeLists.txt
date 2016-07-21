cmake_minimum_required(VERSION 2.8.3)
project(demonstration_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  nav_msgs
  recovery_supervisor_msgs
  roscpp
)

generate_dynamic_reconfigure_options(cfg/DemonstrationLayer.cfg)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES demonstration_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure recovery_supervisor_msgs roscpp
)

include_directories(
  include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
)

add_library(demonstration_layer src/demonstration_layer.cpp)

add_dependencies(demonstration_layer ${PROJECT_NAME}_gencfg)

target_link_libraries(demonstration_layer
   ${catkin_LIBRARIES}
)

add_dependencies(demonstration_layer ${PROJECT_NAME}_gencfg)
add_dependencies(demonstration_layer ${catkin_EXPORTED_TARGETS})
