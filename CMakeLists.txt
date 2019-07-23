cmake_minimum_required(VERSION 2.8.3)
project(stabiliplus)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# set(CMAKE_CXX_FLAGS "-O2")

catkin_package(
  # INCLUDE_DIRS include
  #  LIBRARIES kuroiwa
  #  CATKIN_DEPENDS other_catkin_pkg
  #  DEPENDS system_lib
  )

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

add_library(stability_polytope
  src/stability_polytope.cpp
  src/vertex.cpp
  src/edge.cpp
  src/face.cpp
  src/outervertex.cpp
  src/outeredge.cpp
  src/outerface.cpp
  )
target_link_libraries(stability_polytope glpk)

add_library(robot
  src/robot.cpp
  src/contactPoints.cpp
  )
target_link_libraries(robot tinyxml2)

add_library(experimenter
  src/experimenter.cpp
  )
target_link_libraries(experimenter robot stability_polytope tinyxml2)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}_node experimenter)
