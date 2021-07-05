#ifndef SOLVER_WRAPPER_H_INCLUDE
#define SOLVER_WRAPPER_H_INCLUDE

// standard libraries
#include <iostream>

// libraries
#include <Eigen/Dense>

// homemade includes

enum Solver
{
  GLPK,
  LP_SOLVE,
  GUROBI
};

class SolverWrapper
{
public:
  // ---------- constructor and destructor -------------
  SolverWrapper();
  virtual ~SolverWrapper();

  // ---------- main functions -------------
  virtual void buildProblem(const Eigen::VectorXd & B, const Eigen::MatrixXd &A, const Eigen::MatrixXd &F, const Eigen::VectorXd & f) = 0;
  virtual bool solveProblem() = 0;

  // ---------- getters -----------
  Eigen::Vector3d get_result();
  Eigen::Vector2d get_staticResult();

  // ---------- setters -----------
  virtual void set_searchDirection(const Eigen::Vector3d & searchDir) = 0;
  virtual void set_staticSearchDirection(const Eigen::Vector2d & searchDir) = 0;

protected:
  Eigen::Vector3d m_result;
  Eigen::Vector3d m_searchDirection;
  Eigen::Vector2d m_staticSearchDirection;

  bool m_problemSolved;
};

#endif // SOLVER_WRAPPER_H_INCLUDE
