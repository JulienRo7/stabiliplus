#ifndef GLPK_WRAPPER_H_INCLUDE
#define GLPK_WRAPPER_H_INCLUDE

// standard libraries
#include <iostream>

// libraries
#include <glpk.h>

// homemade includes
#include "wrapper/solver_wrapper.h"

class GlpkWrapper : public SolverWrapper
{
public:
  // ---------- constructor and destructor -------------
  GlpkWrapper();
  ~GlpkWrapper();

  // ---------- main functions -------------
  void buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void buildFactorizedProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void buildOriginalProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f);
  void solveProblem();
  void solveOriginalProblem();
  void solveFactorizedProblem();

  // ---------- getters -----------

  // ---------- setters -----------
  void set_searchDirection(const Eigen::Vector3d & searchDir);
  void set_searchDirectionOriginal(const Eigen::Vector3d & searchDir);
  void set_searchDirectionFactorized(const Eigen::Vector3d & searchDir);
  void set_staticSearchDirection(const Eigen::Vector2d & searchDir);
  void set_staticSearchDirectionOriginal(const Eigen::Vector2d & searchDir);
  void set_staticSearchDirectionFactorized(const Eigen::Vector2d & searchDir);

private:
  glp_prob * m_lp;

  int m_originalNumCols;
  Eigen::MatrixXd m_Q_u;
  Eigen::MatrixXd m_Q_c;
  Eigen::MatrixXd m_R_inv_T_b;
};

#endif // GLPK_WRAPPER_H_INCLUDE
