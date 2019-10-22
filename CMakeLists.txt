cmake_minimum_required(VERSION 3.5)

project(Stabiliplus VERSION 1.0.0)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-O2")


add_subdirectory(lpsolve55 EXCLUDE_FROM_ALL)

find_file(gurobi_lib libgurobi81.so
  PATH $ENV{GUROBI_HOME}/lib/
  )
find_file(gurobi_c++_lib libgurobi_c++.a
  PATH $ENV{GUROBI_HOME}/lib/
  )


### wrapper library ###

add_library(wrappers
  ${PROJECT_SOURCE_DIR}/src/wrapper/solver_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/glpk_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/lpsolve_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/gurobi_wrapper.cpp
  )

target_include_directories(wrappers
  PUBLIC 
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $ENV{GUROBI_HOME}/include
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_compile_features(wrappers
  PRIVATE cxx_std_11
  )

target_link_libraries(wrappers
  PUBLIC
  Eigen3::Eigen
  PRIVATE
  glpk lpsolve55 ${gurobi_c++_lib} ${gurobi_lib}
  )

### contactSet library ###

add_library(contactSet
  ${PROJECT_SOURCE_DIR}/src/contactSet/contactSet.cpp
  ${PROJECT_SOURCE_DIR}/src/contactSet/contactPoints.cpp
  )

target_include_directories(contactSet
  PUBLIC 
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_compile_features(contactSet
  PRIVATE cxx_std_11
  )

target_link_libraries(contactSet
  PUBLIC
  Eigen3::Eigen
  PRIVATE
  tinyxml2
  )

### StabilityPolytope library ###

add_library(StabilityPolytope
  ${PROJECT_SOURCE_DIR}/src/polytope/stabilityPolytope.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/robustStabilityPolytope.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/staticStabilityPolytope.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/staticPoint.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/vertex.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/edge.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/face.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/outervertex.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/outeredge.cpp
  ${PROJECT_SOURCE_DIR}/src/polytope/outerface.cpp
  )

target_include_directories(StabilityPolytope
  PUBLIC 
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_compile_features(StabilityPolytope
  PRIVATE cxx_std_11
  )

target_link_libraries(StabilityPolytope
  PUBLIC
  Eigen3::Eigen
  # contactSet
  wrappers
  PRIVATE
  )

### Main executable ###

add_executable(main
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PROJECT_SOURCE_DIR}/src/experimenter.cpp
  )

target_include_directories(main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_compile_features(main
  PRIVATE cxx_std_11
  )

target_link_libraries(main
  PRIVATE
  contactSet StabilityPolytope tinyxml2)
