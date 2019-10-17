#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef STATIC_STABILITY_POLYTOPE_H_INCLUDE
#define STATIC_STABILITY_POLYTOPE_H_INCLUDE

// standart libraries
#include <iostream>
#include <fstream>
#include <list>
#include <cmath>
#include <algorithm>

// libraries
#include <Eigen/Dense>

// custom libraries
#include "wrapper/glpk_wrapper.h"
#include "wrapper/lpsolve_wrapper.h"
#include "wrapper/gurobi_wrapper.h"
#include "stabiliplus/contactSet.h"

class StaticStabilityPolytope
{
 public:
  StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration = 50, Solver solver = GLPK);
  ~StaticStabilityPolytope();

  // ----- main class methods ------
  void initSolver();
  void solveLP(Eigen::Vector2d const& direction, Eigen::Vector2d &vertex);

  void projectionStabilityPolyhedron();

  Eigen::Vector2d computeOuterVertex(const Eigen::Vector2d& v1, const Eigen::Vector2d& d1, const Eigen::Vector2d& v2, const Eigen::Vector2d& d2);
  Eigen::Vector2d computeSidesNormal(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
  
  // ----- output -----
  void saveResults(std::string file_name);

  // ----- setters -----

  // ----- getters -----

  
 private:
  // model description
  ContactSet m_contactSet;

  // solver
  Solver m_solver;
  SolverWrapper *m_lp;

  // algoritm stop
  int m_maxIterations;
  int m_iteration;

  // algorithm storage
  std::list<Eigen::Vector2d> m_innerVertices;
  std::list<Eigen::Vector2d> m_searchDirections;
  std::list<Eigen::Vector2d> m_outerVertices;
  std::list<Eigen::Vector2d> m_normals;
  std::list<double> m_measures;
};
#endif // STATIC_STABILITY_POLYTOPE_H_INCLUDE
