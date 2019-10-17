#include "wrapper/gurobi_wrapper.h"

GurobiWrapper::GurobiWrapper():
  m_env(), m_lp(m_env)
{
}

GurobiWrapper::~GurobiWrapper()
{
  delete m_vars;
}

void GurobiWrapper::buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f)
{
  m_originalNumCols = A.cols();
  
  /* Factorisation of the problem to reduce the number of unknown and/or constraint and increase the solving of the problem. */
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.transpose());
  
  // auto start = std::chrono::high_resolution_clock::now();
  const Eigen::MatrixXd& Q = qr.householderQ();
  m_Q_c = Q.leftCols(A.rows());//.setLength(qr.nonzeroPivots());
  m_Q_u = Q.rightCols(A.cols()-A.rows());//.setLength(qr.nonzeroPivots());
  m_R_inv_T_b = qr.matrixQR().topRows(A.rows()).transpose().triangularView<Eigen::Lower>().solve(B);

  // auto stop = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "QR decomposition time: " << duration.count() << " microseconds"<< '\n';

  Eigen::MatrixXd F_bis = F*m_Q_u;
  Eigen::VectorXd f_bis = f - (F*m_Q_c)*m_R_inv_T_b;
    
  int const numberOfColumns = F_bis.cols();
  int const numberOfRows = F_bis.rows();
 
  /* Building of the glpk problem using the factorized problem */

  // adding the variables
  double lb[numberOfColumns], ub[numberOfColumns], coef[numberOfColumns];
  std::fill_n(lb, numberOfColumns, -GRB_INFINITY);
  std::fill_n(ub, numberOfColumns, GRB_INFINITY);
  //std::fill_n(coef, numberOfColumns, NULL);
  char type_var[numberOfColumns];
  std::fill_n(type_var, numberOfColumns, GRB_CONTINUOUS);
  std::string names[numberOfColumns];
  // std::fill_n(names, numberOfColumns, NULL);
  m_vars = m_lp.addVars(lb, ub, coef, type_var, names, numberOfColumns);

  // add the constraints
  for (int i = 0; i < numberOfRows; i++)
    {
      GRBLinExpr constraintRow;
      for (int j=0; j<numberOfColumns; j++)
	{
	  constraintRow += F_bis(i,j) * (m_vars[j]);
	}
      // constraintRow.addTerms(F_bis.row(i), m_vars, numberOfColumns);
      
      m_lp.addConstr(constraintRow, GRB_LESS_EQUAL, f_bis(i));
    }

  m_lp.set(GRB_IntParam_OutputFlag, 0);
}

void GurobiWrapper::solveProblem()
{
  m_lp.optimize();

  int numCols = m_lp.get(GRB_IntAttr_NumVars);
  Eigen::VectorXd z(numCols);

  for (int i = 0; i<numCols; ++i)
  {
    z[i]=m_vars[i].get(GRB_DoubleAttr_X);
  }

  const Eigen::VectorXd x = m_Q_c*m_R_inv_T_b + (m_Q_u*z);

  m_result = x.tail(3);
}

  // ---------- getters -----------

  // ---------- setters -----------
void GurobiWrapper::set_searchDirection(const Eigen::Vector3d & searchDir)
{
  m_searchDirection = searchDir;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(3)=searchDir;

  const Eigen::VectorXd c_bis = m_Q_u.transpose()*c;

  GRBLinExpr obj;
  obj.addTerms(c_bis.transpose().data(), m_vars, c_bis.rows());
  m_lp.setObjective(obj, GRB_MAXIMIZE);

  // m_lp.write("test.lp");
}

void GurobiWrapper::set_staticSearchDirection(const Eigen::Vector2d & searchDir)
{
  m_staticSearchDirection = searchDir;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(2)=searchDir;

  const Eigen::VectorXd c_bis = m_Q_u.transpose()*c;

  GRBLinExpr obj;
  obj.addTerms(c_bis.transpose().data(), m_vars, c_bis.rows());
  m_lp.setObjective(obj, GRB_MAXIMIZE);

  // m_lp.write("test.lp");
}
