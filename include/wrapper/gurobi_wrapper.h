#ifndef GUROBI_WRAPPER_H_INCLUDE
#define GUROBI_WRAPPER_H_INCLUDE


//standard libraries
#include <iostream>

// libraries
#include <gurobi_c++.h>

// homemade includes
#include "wrapper/solver_wrapper.h"

class GurobiWrapper: public SolverWrapper
{
 public:
  // ---------- constructor and destructor -------------
  GurobiWrapper();
  ~GurobiWrapper();

  // ---------- main functions -------------
  void buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void solveProblem();

  // ---------- getters -----------


  // ---------- setters -----------
  void set_searchDirection(const Eigen::Vector3d & searchDir);
  void set_staticSearchDirection(const Eigen::Vector2d & searchDir);

 private:
  GRBEnv m_env;
  GRBModel m_lp;

  GRBVar* m_vars;

  int m_originalNumCols;
  Eigen::MatrixXd m_Q_u;
  Eigen::MatrixXd m_Q_c;
  Eigen::MatrixXd m_R_inv_T_b;
};

#endif // GUROBI_WRAPPER_H_INCLUDE
