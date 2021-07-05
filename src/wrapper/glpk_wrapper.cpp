#include "wrapper/glpk_wrapper.h"

GlpkWrapper::GlpkWrapper()
{
  m_lp = glp_create_prob();
}

GlpkWrapper::~GlpkWrapper()
{
  glp_delete_prob(m_lp);
}

void GlpkWrapper::buildProblem(const Eigen::VectorXd & B, const Eigen::MatrixXd & A, const Eigen::MatrixXd & F, const Eigen::VectorXd & f)
{
  buildOriginalProblem(B, A, F, f);
  //buildFactorizedProblem(B, A, F, f);
}

void GlpkWrapper::buildFactorizedProblem(const Eigen::VectorXd & B, const Eigen::MatrixXd & A, const Eigen::MatrixXd & F, const Eigen::VectorXd & f)
{
  m_originalNumCols = A.cols();

  /* Factorisation of the problem to reduce the number of unknown and/or constraint and increase the solving of the
   * problem. */
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.transpose());

  // auto start = std::chrono::high_resolution_clock::now();
  const Eigen::MatrixXd & Q = qr.householderQ();
  m_Q_c = Q.leftCols(A.rows()); //.setLength(qr.nonzeroPivots());
  m_Q_u = Q.rightCols(A.cols() - A.rows()); //.setLength(qr.nonzeroPivots());
  m_R_inv_T_b = qr.matrixQR().topRows(A.rows()).transpose().triangularView<Eigen::Lower>().solve(B);

  // auto stop = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "QR decomposition time: " << duration.count() << " microseconds"<< '\n';

  Eigen::MatrixXd F_bis = F * m_Q_u;
  Eigen::VectorXd f_bis = f - (F * m_Q_c) * m_R_inv_T_b;

  int const numberOfColumns = F_bis.cols();
  int const numberOfRows = F_bis.rows();

  /* Building of the glpk problem using the factorized problem */
  glp_set_obj_dir(m_lp, GLP_MAX); // The objective here is to maximize
  glp_add_rows(m_lp, numberOfRows);

  for(int i = 0; i < numberOfRows; i++)
  {
    glp_set_row_bnds(m_lp, i + 1, GLP_UP, 0.0, f_bis[i]);
  }

  glp_add_cols(m_lp, numberOfColumns);
  for(int i = 0; i < numberOfColumns; ++i)
  {
    glp_set_col_bnds(m_lp, i + 1, GLP_FR, -100.0, 100.0);
  }

  int ia[1 + numberOfRows * numberOfColumns], ja[1 + numberOfRows * numberOfColumns];
  double ar[1 + numberOfRows * numberOfColumns];

  for(int i = 0; i < numberOfRows; ++i)
  {
    for(int j = 0; j < numberOfColumns; ++j)
    {
      ia[1 + i * numberOfColumns + j] = 1 + i;
      ja[1 + i * numberOfColumns + j] = 1 + j;
      ar[1 + i * numberOfColumns + j] = F_bis(i, j);
    }
  }

  glp_load_matrix(m_lp, numberOfRows * numberOfColumns, ia, ja, ar);
  glp_term_out(GLP_OFF);

  // glp_write_lp(m_lp, NULL, "export_lp.txt");
}

void GlpkWrapper::buildOriginalProblem(const Eigen::VectorXd & B, const Eigen::MatrixXd A, const Eigen::MatrixXd & F, const Eigen::VectorXd & f)
{
  m_originalNumCols = A.cols();

  int const numberOfColumns = A.cols();
  int const numberOfRows = A.rows() + F.rows();

  /* Building of the glpk problem using the original problem */
  glp_set_obj_dir(m_lp, GLP_MAX); // The objective here is to maximize
  glp_add_rows(m_lp, numberOfRows);

  for(int i = 0; i < numberOfRows; i++)
  {
    if(i < A.rows())
    {
      glp_set_row_bnds(m_lp, i + 1, GLP_FX, B[i], B[i]);
    }
    else
    {
      glp_set_row_bnds(m_lp, i + 1, GLP_UP, 0.0, f[i - A.rows()]);
    }
  }

  glp_add_cols(m_lp, numberOfColumns);
  for(int i = 0; i < numberOfColumns; ++i)
  {
    glp_set_col_bnds(m_lp, i + 1, GLP_FR, -100.0, 100.0);
  }

  int ia[1 + numberOfRows * numberOfColumns], ja[1 + numberOfRows * numberOfColumns];
  double ar[1 + numberOfRows * numberOfColumns];

  for(int i = 0; i < numberOfRows; ++i)
  {
    for(int j = 0; j < numberOfColumns; ++j)
    {
      ia[1 + i * numberOfColumns + j] = 1 + i;
      ja[1 + i * numberOfColumns + j] = 1 + j;
      if(i < A.rows())
      {
        ar[1 + i * numberOfColumns + j] = A(i, j);
      }
      else
      {
        ar[1 + i * numberOfColumns + j] = F(i - A.rows(), j);
      }
    }
  }

  glp_load_matrix(m_lp, numberOfRows * numberOfColumns, ia, ja, ar);
  glp_term_out(GLP_OFF);

  // glp_write_lp(m_lp, NULL, "export_lp.txt");
}

bool GlpkWrapper::solveProblem()
{
  return solveOriginalProblem();
  //solveFactorizedProblem();
}

bool GlpkWrapper::solveOriginalProblem()
{
  auto ret = glp_simplex(m_lp, NULL);
  //std::cout <<   << std::endl;
  if (ret == 0)
  {
    int numCols = glp_get_num_cols(m_lp);
    Eigen::VectorXd z(numCols);

    for(int i = 0; i < numCols; ++i)
    {
      z[i] = glp_get_col_prim(m_lp, i + 1);
    }

    m_result = z.tail(3);

    return true;
  }
  else
  {
    m_result << 0.0, 0.0, 0.0;
    std::cerr << "The solver failed with error: "<< ret << std::endl;
    return false;
  }

  
}

void GlpkWrapper::solveFactorizedProblem()
{
  glp_simplex(m_lp, NULL);

  int numCols = glp_get_num_cols(m_lp);
  Eigen::VectorXd z(numCols);

  for(int i = 0; i < numCols; ++i)
  {
    z[i] = glp_get_col_prim(m_lp, i + 1);
  }

  const Eigen::VectorXd x = m_Q_c * m_R_inv_T_b + (m_Q_u * z);
  // std::cout << "LP solution: " << x.transpose() << std::endl;
  m_result = x.tail(3);
}

// ---------- getters -----------

// ---------- setters -----------
void GlpkWrapper::set_searchDirection(const Eigen::Vector3d & searchDir)
{
  m_searchDirection = searchDir;
  set_searchDirectionOriginal(m_searchDirection);
  //set_searchDirectionFactorized(m_searchDirection);
}

void GlpkWrapper::set_searchDirectionOriginal(const Eigen::Vector3d & searchDir)
{
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(3) = searchDir;

  for(int i = 0; i < c.size(); ++i)
  {
    glp_set_obj_coef(m_lp, i + 1, c[i]);
  }
}

void GlpkWrapper::set_searchDirectionFactorized(const Eigen::Vector3d & searchDir)
{

  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(3) = searchDir;

  const Eigen::VectorXd c_bis = m_Q_u.transpose() * c;

  for(int i = 0; i < c_bis.size(); ++i)
  {
    glp_set_obj_coef(m_lp, i + 1, c_bis[i]);
  }
}

void GlpkWrapper::set_staticSearchDirection(const Eigen::Vector2d & searchDir)
{
  m_staticSearchDirection = searchDir;
  set_staticSearchDirectionOriginal(searchDir);
  //set_staticSearchDirectionFactorized(searchDir);
}

void GlpkWrapper::set_staticSearchDirectionOriginal(const Eigen::Vector2d & searchDir)
{
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(2) = searchDir;

  for(int i = 0; i < c.size(); ++i)
  {
    glp_set_obj_coef(m_lp, i + 1, c[i]);
  }
}

void GlpkWrapper::set_staticSearchDirectionFactorized(const Eigen::Vector2d & searchDir)
{
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(2) = searchDir;

  const Eigen::VectorXd c_bis = m_Q_u.transpose() * c;

  for(int i = 0; i < c_bis.size(); ++i)
  {
    glp_set_obj_coef(m_lp, i + 1, c_bis[i]);
  }
}
