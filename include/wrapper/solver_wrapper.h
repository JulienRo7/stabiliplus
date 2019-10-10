#ifndef SOLVER_WRAPPER_H_INCLUDE
#define SOLVER_WRAPPER_H_INCLUDE


//standard libraries
#include <iostream>

// libraries
#include <Eigen/Dense>

// homemade includes

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

  // ---------- setters -----------
  virtual void set_searchDirection(const Eigen::Vector3d & searchDir) = 0;

 protected:
  Eigen::Vector3d m_result;
  Eigen::Vector3d m_searchDirection;

  bool m_problemSolved;
};

#endif // SOLVER_WRAPPER_H_INCLUDE