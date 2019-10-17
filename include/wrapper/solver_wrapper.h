#ifndef SOLVER_WRAPPER_H_INCLUDE
#define SOLVER_WRAPPER_H_INCLUDE


//standard libraries
#include <iostream>

// libraries
#include <Eigen/Dense>

// homemade includes

enum Solver {
  GLPK,
  LP_SOLVE,
  GUROBI
};

class SolverWrapper
{
 public:
  // ---------- constructor and destructor -------------
  SolverWrapper();
  ~SolverWrapper();

  // ---------- main functions -------------
  virtual void buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f) = 0;
  virtual void solveProblem() = 0;

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
