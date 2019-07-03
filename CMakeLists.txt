project(stabiliplus)

cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-O2")
add_compile_options(-std=c++11)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(robot
  ${PROJECT_SOURCE_DIR}/src/robot.cpp
  ${PROJECT_SOURCE_DIR}/src/contactPoints.cpp
  ${PROJECT_SOURCE_DIR}/src/vertex.cpp
  ${PROJECT_SOURCE_DIR}/src/edge.cpp
  ${PROJECT_SOURCE_DIR}/src/face.cpp
  )
target_link_libraries(robot glpk tinyxml2)

add_executable(main
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  )
target_link_libraries(main robot)
