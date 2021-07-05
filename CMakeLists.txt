cmake_minimum_required(VERSION 3.10)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

project(Stabiliplus VERSION 1.0.0)
find_package(Eigen3 REQUIRED)
find_package(sch-core REQUIRED)
find_package(eigen-quadprog REQUIRED)
# threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


# add_subdirectory(lpsolve55)
# find_package(lpsolve55 REQUIRED)
# find_package(lpsolve55 REQUIRED)
# add_required_dependency(lpsolve55)

#find_file(gurobi_lib libgurobi90.so
#  PATH $ENV{GUROBI_HOME}/lib/
#  )
#find_file(gurobi_c++_lib libgurobi_c++.a
#  PATH $ENV{GUROBI_HOME}/lib/
#  )

### wrapper library ###

add_library(wrappers SHARED
  ${PROJECT_SOURCE_DIR}/src/wrapper/solver_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/glpk_wrapper.cpp
  #${PROJECT_SOURCE_DIR}/src/wrapper/lpsolve_wrapper.cpp
  #${PROJECT_SOURCE_DIR}/src/wrapper/gurobi_wrapper.cpp
  )

target_include_directories(wrappers
  PUBLIC 
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lpsolve55/src>
  #$ENV{GUROBI_HOME}/include
  # PRIVATE
  #${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_link_libraries(wrappers
  PUBLIC
  Eigen3::Eigen
  PRIVATE
  glpk 
  #${gurobi_c++_lib} 
  #${gurobi_lib}
  #lpsolve55::lpsolve55 
  )

### ContactSet library ###

add_library(ProblemDescriptor SHARED
  ${PROJECT_SOURCE_DIR}/src/problemDescriptor/problemDescriptor.cpp
  ${PROJECT_SOURCE_DIR}/src/problemDescriptor/contactSet.cpp
  ${PROJECT_SOURCE_DIR}/src/problemDescriptor/contactPoints.cpp
  )

target_include_directories(ProblemDescriptor
  PUBLIC 
  $<INSTALL_INTERFACE:include>    
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )


target_link_libraries(ProblemDescriptor
  PUBLIC
  Eigen3::Eigen
  PRIVATE
  tinyxml2
  )

### StabilityPolytope library ###

add_library(StabilityPolytope SHARED
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
  glpk 
  )

target_link_libraries(StabilityPolytope
  PUBLIC
  Eigen3::Eigen
  wrappers
  PRIVATE
  )

### Main executable ###

add_executable(main
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  ${PROJECT_SOURCE_DIR}/src/experimenter.cpp
  ${PROJECT_SOURCE_DIR}/src/CoMQP.cpp
  ${PROJECT_SOURCE_DIR}/src/PointProjector.cpp
  )

target_include_directories(main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

target_link_libraries(main
  PRIVATE
  ProblemDescriptor StabilityPolytope tinyxml2
  Threads::Threads
  sch-core::sch-core
  eigen-quadprog::eigen-quadprog
  -lm
  -ltbb
  )



# set_target_properties(main
#   PROPERTIES
#   CXX_STANDART 17
#   CXX_EXTENSIONS OFF
#   )
# target_compile_features(main
#   PRIVATE
#   cxx_std_17
#   )

#### Exporting targets and installing ####

include(GNUInstallDirs)
install(TARGETS StabilityPolytope ProblemDescriptor wrappers
    EXPORT Stabiliplus-export
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/
    )

install(EXPORT Stabiliplus-export
  FILE
  StabiliplusConfig.cmake
  NAMESPACE
  Stabiliplus::
  DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/Stabiliplus
  )

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
  )

