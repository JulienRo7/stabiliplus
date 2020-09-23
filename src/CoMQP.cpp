#include "CoMQP.h"

#include <mc_rtc/logging.h>

#include <cassert> 

CoMQP::CoMQP()
{

}

CoMQP::CoMQP(std::shared_ptr<ContactSet> contactSet)
{
  setContactSet(contactSet);
}

void CoMQP::setContactSet(std::shared_ptr<ContactSet> contactSet)
{
  contactSet_ = contactSet;
}

void CoMQP::addCoMRegionPlanes(std::vector<Eigen::Vector4d> planes)
{
  comRegionPlanes_ = planes;
}

void CoMQP::updateProblem()
{
  contactSet_->update();
  int nrVar, nrEq, nrInEq;
  if (!contactSet_->hasConstrainedContact() or !considerConstrainedContacts_)
    {
        nrVar = contactSet_->globCols();
	nrEq = 6 * contactSet_->get_numberOfAccelerations();
	nrInEq = contactSet_->globRows(); // + comRegionPlanes_.size();
    }
  else
    {
      std::cout << "The contactSet has constrained contacts!" << std::endl;
      int numContacts = contactSet_->numberOfContacts();
      int numConstrainedContacts = contactSet_->numberConstrainedContacts();
      int numFrictionSides = contactSet_->frictionSides();
      int numAccelerations = contactSet_->get_numberOfAccelerations();

      // number of variable = non-conservative case forces + conservative case forces + com position
      nrVar = (3 * numContacts) * numAccelerations + (3 * (numContacts-numConstrainedContacts)) * numAccelerations + 3;
      // number of equalities = non consercative case + conservative case
      nrEq = 6 * numAccelerations +  6 * numAccelerations;
      // number of inequalities = number of forces (conervative and non conservative) *( number of friction sides  + 2 for upper and lower limits)
      // + 6 for com position boundaries
      nrInEq = (numContacts + numContacts-numConstrainedContacts) * (numFrictionSides + 2) + 6 ; // + comRegionPlanes_.size();

      // std::cout << "nrVar: " << nrVar << " nrEq: " << nrEq << " nrInEq: " << nrInEq << std::endl;
    }
  
  solver_.problem(nrVar, nrEq, nrInEq);
}

bool CoMQP::solve(Eigen::Vector3d currentCoM)
{
  updateProblem();
  if (!contactSet_->hasConstrainedContact() or !considerConstrainedContacts_)
    {
      return solveNonConstrained(currentCoM);
    }
  else
    {
      //return solveNonConstrained(currentCoM);
      return solveConstrained(currentCoM);
    }
}

bool CoMQP::solveNonConstrained(Eigen::Vector3d currentCoM)
{
  const int nrVar = contactSet_->globCols();
  const int nrEq = 6 * contactSet_->get_numberOfAccelerations();
  const int nrInEq = contactSet_->globRows(); // + comRegionPlanes_.size();

  auto A = contactSet_->getMatrixA();
  auto b = contactSet_->getVectorB();

  auto F = contactSet_->getFrictionF();
  auto f = contactSet_->getFrictionVectorf();

  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(nrVar, nrVar);
  P.bottomRightCorner<3,3>() = 100 * Eigen::Matrix3d::Identity();

  auto Ydes = contactSet_->Ydes();
  Eigen::VectorXd q(nrVar);
  q = contactSet_->Ydes();

  q.tail<3>() = currentCoM;
  q = - P * q;

  bool ret = solver_.solve(P, q, A, b, F, f);
  return ret;
}

void displayMatrix01(Eigen::MatrixXd mat)
{
  double eps = 0.000001;
  for (int i=0; i<mat.rows(); i++)
    {
      for (int j=0; j<mat.cols(); j++)
	{
	  std::cout << ((std::abs(mat(i, j))<=eps)?0:1);
	}
      std::cout << std::endl;
    }
}

bool CoMQP::solveConstrained(Eigen::Vector3d currentCoM)
{
  // std::cout << "Starting to solve the Constrained Case" << std::endl;
  
  int numContacts = contactSet_->numberOfContacts();
  int numConstrainedContacts = contactSet_->numberConstrainedContacts();
  int numFrictionSides = contactSet_->frictionSides();
  int numAccelerations = contactSet_->get_numberOfAccelerations();
  
  // // number of variable = non-conservative case forces + conservative case forces + com position
  int nrVar = (3 * numContacts) * numAccelerations + (3 * (numContacts-numConstrainedContacts)) * numAccelerations + 3;
  // number of equalities = non consercative case + conservative case
  int nrEq = 6 * numAccelerations +  6 * numAccelerations;
  // number of inequalities = number of forces (conervative and non conservative) *( number of friction sides  + 2 for upper and lower limits)
  // + 6 for com position boundaries
  int nrInEq = (numContacts + numContacts-numConstrainedContacts) * (numFrictionSides + 2) + 6 ; // + comRegionPlanes_.size();xs

  int numSubEq = 6 * numAccelerations;
  int numNonConservativeForcesVar = (3 * numContacts) * numAccelerations;
  int numConservativeForcesVar = (3 * (numContacts-numConstrainedContacts)) * numAccelerations;

  // const int nrVar = contactSet_->globCols();
  // assert( nrVar == numNonConservativeForcesVar + 3);
  // const int nrEq = 6 * contactSet_->get_numberOfAccelerations();
  // assert(nrEq == numSubEq);
  // const int nrInEq = contactSet_->globRows(); // + comRegionPlanes_.size();
  // assert(nrInEq == numContacts * (numFrictionSides + 2) + 6);
  
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nrEq, nrVar);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(nrEq);
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(nrInEq, nrVar);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(nrInEq);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(nrVar);

  // std::cout << "F:" << std::endl;
  // displayMatrix01(F);
  
  // non conservative case
  auto A_1 = contactSet_->getMatrixA();
  auto b_1 = contactSet_->getVectorB();

  auto F_1 = contactSet_->getFrictionF();
  auto f_1 = contactSet_->getFrictionVectorf();

  auto Ydes_1 = contactSet_->Ydes();
  
  // removing the contact forces that are non conservatives
  std::vector<std::string> names = {"RightHandPad_0", "RightHandPad_1", "RightHandPad_2", "RightHandPad_3", "RightHandPad_4"};

  ContactSet nonConstrainedContactSet(*contactSet_);
  
  for (auto name: names)
    {
      if (nonConstrainedContactSet.hasContactNamed(name))
  	{
  	  nonConstrainedContactSet.removeContact(name);
  	}
    }
  
  // nonConstrainedContactSet.showContactSet();
  nonConstrainedContactSet.update();

  auto A_2 = nonConstrainedContactSet.getMatrixA();
  auto b_2 = nonConstrainedContactSet.getVectorB();
  
  auto F_2 = nonConstrainedContactSet.getFrictionF();  
  auto f_2 = nonConstrainedContactSet.getFrictionVectorf();

  auto Ydes_2 = nonConstrainedContactSet.Ydes();
  
  // std::cout << "A: " << A.rows() << " x " << A.cols() << std::endl;
  // std::cout << "A_1: " << A_1.rows() << " x " << A_1.cols() << std::endl;
  // std::cout << "A_2: " << A_2.rows() << " x " << A_2.cols() << std::endl;

  A.topLeftCorner(A_1.rows(), A_1.cols()-3) = A_1.leftCols(A_1.cols()-3);
  A.topRightCorner(A_1.rows(), 3) = A_1.rightCols(3);
  A.bottomRightCorner(A_2.rows(), A_2.cols()) = A_2;
  //displayMatrix01(A);
  
  
  // std::cout << "b: " << b.size() << std::endl;
  // std::cout << "b_1: " << b_1.size() << std::endl;
  // std::cout << "b_2: " << b_2.size() << std::endl;
  
  b.head(b_1.size()) = b_1;
  b.tail(b_2.size()) = b_2;
  // std::cout << "b: " << b << std::endl;

  // std::cout << "F: " << F.rows() << " x " << F.cols() << std::endl;
  // std::cout << "F_1: " << F_1.rows() << " x " << F_1.cols() << std::endl;
  // std::cout << "F_2: " << F_2.rows() << " x " << F_2.cols() << std::endl;

  F.topLeftCorner(F_1.rows(), F_1.cols()-3) = F_1.leftCols(F_1.cols()-3);
  F.bottomRightCorner(F_2.rows(), F_2.cols()) = F_2;
  //displayMatrix01(F);

  // std::cout << "f: " << f.size() << std::endl;
  // std::cout << "f_1: " << f_1.size() << std::endl;
  // std::cout << "f_2: " << f_2.size() << std::endl;
							       
  f.head(f_1.size()-6) = f_1.head(f_1.size()-6);
  f.tail(f_2.size()) = f_2;

  // std::cout << "q: " << q.size() << std::endl;
  // std::cout << "Ydes_1: " << Ydes_1.size() << std::endl;
  // std::cout << "Ydes_2: " << Ydes_2.size() << std::endl;

  q.head(Ydes_1.size()) = Ydes_1;
  q.tail(Ydes_2.size()) = Ydes_2;
  
  // setting up the weight matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(nrVar, nrVar);
  P.block(numConservativeForcesVar, numConservativeForcesVar, 3*numConstrainedContacts, 3*numConstrainedContacts) = 100 * Eigen::MatrixXd::Identity(3*numConstrainedContacts, 3 * numConstrainedContacts);
  P.bottomRightCorner<3,3>() = 1000 * Eigen::Matrix3d::Identity();
  // std::cout << "P: \n" << P << std::endl; 
  // set the CoM objective
  q.tail<3>() = currentCoM;
  q = - P * q;
    
  bool ret = solver_.solve(P, q, A, b, F, f);
  
  // std::cout << (ret?"Success":"Fail") << std::endl;
  // std::cout << "Constrained case solved" << std::endl;
  return ret;
}

const Eigen::VectorXd & CoMQP::resultVector() const
{
  auto res = solver_.result();
  // int n = contactSet_->globCols()-3;
  // Eigen::MatrixXd dispRes = Eigen::MatrixXd::Zero(n, 3);

  // dispRes.leftCols<1>() = res.head(n);
  // dispRes.col(1) = res.segment(n, n);
  // dispRes.bottomRightCorner(3, 1) = res.tail(3);

  // std::cout << dispRes << std::endl;
  
  return solver_.result();
}

Eigen::Vector3d CoMQP::resultCoM() const
{
  
  const Eigen::VectorXd & r = resultVector();
  // std::cout << "Result of the QP: \n" << r.transpose() << std::endl;
  Eigen::Vector3d res;
  res = r.tail<3>();
  return res;
}

int CoMQP::errorCode() const
{
  return solver_.fail();
}

