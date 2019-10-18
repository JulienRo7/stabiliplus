#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef STATIC_STABILITY_POLYTOPE_H_INCLUDE
#define STATIC_STABILITY_POLYTOPE_H_INCLUDE

// standart libraries
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

// libraries
#include <Eigen/Dense>

// custom libraries
#include "wrapper/glpk_wrapper.h"
#include "wrapper/lpsolve_wrapper.h"
#include "wrapper/gurobi_wrapper.h"
#include "stabiliplus/contactSet.h"
#include "stabiliplus/staticPoint.h"

class StaticStabilityPolytope
{
 public:
  StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration = 50, double maxError = 1,  Solver solver = GLPK);
  ~StaticStabilityPolytope();

  // ----- main class methods ------
  void initSolver();
  void solveLP(Eigen::Vector2d const& direction, Eigen::Vector2d &vertex);

  void projectionStabilityPolyhedron();

  bool stopCriterion() const; // return true when the algorithm has to stop
  
  // ----- output -----
  void saveResults(std::string file_name);
  void showPointsNeighbours();
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
  double m_maxError;
  int m_iteration;
  double m_error;

  // algorithm storage
  std::vector<std::shared_ptr<StaticPoint>> m_points;
};
#endif // STATIC_STABILITY_POLYTOPE_H_INCLUDE
