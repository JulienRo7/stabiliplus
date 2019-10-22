#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef STABILITY_POLYTOPE_H_INCLUDE
#define STABILITY_POLYTOPE_H_INCLUDE

// standard libraries
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <chrono>
#include <memory>

// libraries
#include <Eigen/Dense>


// custom libraries
#include "wrapper/glpk_wrapper.h"
#include "wrapper/lpsolve_wrapper.h"
#include "wrapper/gurobi_wrapper.h"
#include "contactSet/contactSet.h"


class StabilityPolytope
{
public:
  // ----------- constructors and destructor ----------
  StabilityPolytope(ContactSet contactSet, int maxIteration, double maxError, Solver solveType);
  ~StabilityPolytope();
  
  // ----------- main class methods ----------
  virtual void initSolver() = 0; 
  // virtual void solveLP(Eigen::Vector3d const& direction, Eigen::Vector3d &vertex) = 0;

  virtual void projectionStabilityPolyhedron() = 0;

  bool stopCriterion() const; // return true when the algorithm must stop

  // ----------- output and display functions ----------
  virtual void writeToStream(std::ofstream& stream) const = 0;

  // ----------- getters ----------
  double LPTime() const;
  double initTime() const;
  double structTime() const;
  
  ContactSet* contactSet();
  Solver solverType() const;

  // ----------- setters ----------
  void maxIteration(int maxIteration);

  // ---------- static functions ---------

 protected:
  // robot:
  ContactSet m_contactSet; // for now the contact set should not change
  
  // attributes used for the LP problem
  Solver m_solverType;
  SolverWrapper *m_lp;

  // projection algorithm stop criterion
  int m_iteration;
  double m_error;

  // options
  int m_maxIteration;
  double m_maxError;

  // time measures
  double m_LPTime;
  double m_initTime;
  double m_structTime;
  
}; // class StabilityPolytope

#endif // STABILITY_POLYTOPE_H_INCLUDE
