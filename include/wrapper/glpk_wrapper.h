#ifndef GLPK_WRAPPER_H_INCLUDE
#define GLPK_WRAPPER_H_INCLUDE


//standard libraries
#include <iostream>

// libraries
#include <glpk.h>

// homemade includes
#include "wrapper/solver_wrapper.h"

class GlpkWrapper: public SolverWrapper
{
 public:
  // ---------- constructor and destructor -------------
  GlpkWrapper();
  ~GlpkWrapper();

  // ---------- main functions -------------
  void buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void solveProblem();

  // ---------- getters -----------


  // ---------- setters -----------
  void set_searchDirection(const Eigen::Vector3d & searchDir);

 private:
  glp_prob *m_lp;

  int m_originalNumCols;
  Eigen::MatrixXd m_Q_u;
  Eigen::MatrixXd m_Q_c;
  Eigen::MatrixXd m_R_inv_T_b;
};

#endif // GLPK_WRAPPER_H_INCLUDE
