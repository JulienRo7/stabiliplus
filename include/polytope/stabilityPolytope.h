#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#pragma once

// standard libraries
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// libraries
#include <Eigen/Dense>

// custom libraries
#include "wrapper/glpk_wrapper.h"
//#include "wrapper/lpsolve_wrapper.h"
//#include "wrapper/gurobi_wrapper.h"

#include "problemDescriptor/contactSet.h"
#include "problemDescriptor/problemDescriptor.h"

class StabilityPolytope
{
public:
  // ----------- constructors and destructor ----------
  // StabilityPolytope(ContactSet contactSet, int maxIteration, double maxError, Solver solveType);
  StabilityPolytope(std::shared_ptr<ProblemDescriptor> inputPD, int maxIteration, double maxError, Solver solveType);
  virtual ~StabilityPolytope();

  // ----------- main class methods ----------
  virtual void initSolver() = 0;
  // virtual void solveLP(Eigen::Vector3d const& direction, Eigen::Vector3d &vertex) = 0;

  virtual void projectionStabilityPolyhedron() = 0;

  bool stopCriterion() const; // return true when the algorithm must stop

  // ----------- output and display functions ----------
  virtual void writeToStream(std::ofstream & stream) const = 0;
  virtual std::vector<Eigen::Vector4d> constraintPlanes() const = 0;
  virtual Eigen::Vector3d baryPoint() const = 0;

  // ----------- getters ----------
  double LPTime() const;
  double initTime() const;
  double structTime() const;

  virtual inline std::shared_ptr<ProblemDescriptor> problemDescriptor()
  {
    return m_pdPtr;
  }
  Solver solverType() const;

  // ----------- setters ----------
  void maxIteration(int maxIteration);

  // ---------- static functions ---------

protected:
  // robot:
  std::shared_ptr<ProblemDescriptor> m_pdPtr; // for now the contact set should not change

  // attributes used for the LP problem
  Solver m_solverType;
  SolverWrapper * m_lp;

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
