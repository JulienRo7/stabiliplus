project(stabiliplus)

cmake_minimum_required(VERSION 2.8)
# message(${CMAKE_VERSION})

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

foreach(loop_var ${CMAKE_MODULE_PATH})
  message(path: ${loop_var})
endforeach(loop_var)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "-O2")
add_compile_options(-std=c++11)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(/opt/gurobi811/linux64/include)

add_subdirectory(lpsolve55 EXCLUDE_FROM_ALL)

# looking for the gurobi file: this part can be improved...
find_file(gurobi_lib libgurobi81.so
  PATH /opt/gurobi811/linux64/lib/
  )
# message(${gurobi_lib})
find_file(gurobi_c++_lib libgurobi_c++.a
  PATH /opt/gurobi811/linux64/lib/
  )
# message(${gurobi_c++_lib})

add_library(wrappers
  ${PROJECT_SOURCE_DIR}/src/wrapper/solver_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/glpk_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/lpsolve_wrapper.cpp
  ${PROJECT_SOURCE_DIR}/src/wrapper/gurobi_wrapper.cpp
  )
target_link_libraries(wrappers
  glpk lpsolve55 ${gurobi_c++_lib} ${gurobi_lib}
  )

add_library(stability_polytope
${PROJECT_SOURCE_DIR}/src/stability_polytope.cpp
${PROJECT_SOURCE_DIR}/src/vertex.cpp
${PROJECT_SOURCE_DIR}/src/edge.cpp
${PROJECT_SOURCE_DIR}/src/face.cpp
${PROJECT_SOURCE_DIR}/src/outervertex.cpp
${PROJECT_SOURCE_DIR}/src/outeredge.cpp
${PROJECT_SOURCE_DIR}/src/outerface.cpp
)
target_link_libraries(stability_polytope wrappers)

add_library(staticStabilityPolytope
  ${PROJECT_SOURCE_DIR}/src/staticStabilityPolytope.cpp
  ${PROJECT_SOURCE_DIR}/src/staticPoint.cpp
  )
target_link_libraries(staticStabilityPolytope wrappers)

add_library(contactSet
${PROJECT_SOURCE_DIR}/src/contactSet.cpp
${PROJECT_SOURCE_DIR}/src/contactPoints.cpp
)
target_link_libraries(contactSet tinyxml2)


add_library(experimenter
${PROJECT_SOURCE_DIR}/src/experimenter.cpp
)
target_link_libraries(experimenter contactSet stability_polytope staticStabilityPolytope tinyxml2)

add_executable(main
  ${PROJECT_SOURCE_DIR}/src/main.cpp
  )
target_link_libraries(main experimenter)
