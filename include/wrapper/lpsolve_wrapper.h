#ifndef LPSOLVE_WRAPPER_H_INCLUDE
#define LPSOLVE_WRAPPER_H_INCLUDE


//standard libraries
#include <iostream>

// libraries
#include <lp_lib.h>
//#include <lp_solve/lp_lib.h>

// homemade includes
#include "wrapper/solver_wrapper.h"

class LPSolveWrapper: public SolverWrapper
{
 public:
  // ---------- constructor and destructor -------------
  LPSolveWrapper();
  ~LPSolveWrapper();

  // ---------- main functions -------------
  void buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void solveProblem();

  // ---------- getters -----------


  // ---------- setters -----------
  void set_searchDirection(const Eigen::Vector3d & searchDir);
  void set_staticSearchDirection(const Eigen::Vector2d & searchDir);

 private:
  lprec *m_lp;

  int m_originalNumCols;
  Eigen::MatrixXd m_Q_u;
  Eigen::MatrixXd m_Q_c;
  Eigen::MatrixXd m_R_inv_T_b;
};

#endif // LPSOL_WRAPPER_H_INCLUDE
