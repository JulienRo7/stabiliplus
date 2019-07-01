project(stabiliplus)

find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/includes)

add_executable(main
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PROJECT_SOURCE_DIR}/src/robot.cpp
  ${PROJECT_SOURCE_DIR}/src/contactPoints.cpp
  ${PROJECT_SOURCE_DIR}/src/vertex.cpp
  ${PROJECT_SOURCE_DIR}/src/edge.cpp
  ${PROJECT_SOURCE_DIR}/src/face.cpp
  )

target_link_libraries(main glpk tinyxml2)
