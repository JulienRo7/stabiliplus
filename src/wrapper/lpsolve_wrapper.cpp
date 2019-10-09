#include "wrapper/lpsolve_wrapper.h"

LPSolveWrapper::LPSolveWrapper()
{
  //  m_lp = make_lp(0,0);
}

LPSolveWrapper::~LPSolveWrapper()
{
  delete_lp(m_lp);
}

void LPSolveWrapper::buildProblem(Eigen::VectorXd B, Eigen::MatrixXd A, Eigen::MatrixXd F, Eigen::VectorXd f)
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

  /* Building of the lp solve problem using the factorized problem */
  m_lp = make_lp(0,numberOfColumns);
  resize_lp(m_lp, numberOfRows, numberOfColumns);
 
  Eigen::VectorXd constraintRow = Eigen::VectorXd::Zero(numberOfColumns+1);
  // std::cout << "constraintRow size :"<< constraintRow.rows() << "x" << constraintRow.cols() << std::endl;
  set_add_rowmode(m_lp, true); 
  set_obj_fn(m_lp, constraintRow.data()); // set the objective function to 0 in the first place
  for (int i=0; i<numberOfRows; i++)
  {
    
    constraintRow.tail(numberOfColumns) = F_bis.row(i).transpose();
    
    add_constraintex(m_lp, numberOfColumns, constraintRow.data(), NULL, 1, f_bis(i));
  }
  set_add_rowmode(m_lp, false);
  
  double const infinity = get_infinite(m_lp);
  
  for (int i = 0; i<numberOfColumns; i++)
  {
    // by default the lp_solve bounds are set to [0, inf[
    set_bounds(m_lp, i+1, -infinity, infinity);
  }
  
  set_maxim(m_lp);
  set_verbose(m_lp, IMPORTANT);
  // print_lp(m_lp);
}

void LPSolveWrapper::solveProblem()
{
  solve(m_lp);


  int numCols = get_Ncolumns(m_lp);
  int numRows = get_Nrows(m_lp);
  
  double lpPrimal[1+numRows+numCols];
  get_primal_solution(m_lp, lpPrimal);

  Eigen::VectorXd z(numCols);
  
  for (int i = 0; i<numCols; ++i)
  {
    z[i]=lpPrimal[1+numRows+i];
  }
  const Eigen::VectorXd x = m_Q_c*m_R_inv_T_b + (m_Q_u*z);

  m_result = x.tail(3);
}

  // ---------- getters -----------

  // ---------- setters -----------
void LPSolveWrapper::set_searchDirection(const Eigen::Vector3d & searchDir)
{
  m_searchDirection = searchDir;
  Eigen::VectorXd c = Eigen::VectorXd::Zero(m_originalNumCols);
  c.tail(3)=searchDir;

  Eigen::VectorXd c_bis = Eigen::VectorXd::Zero(get_Ncolumns(m_lp)+1);
  c_bis.tail(get_Ncolumns(m_lp)) = m_Q_u.transpose()*c;
  
  set_obj_fnex(m_lp, c_bis.rows(), c_bis.data(), NULL); 
}
