#include "problemDescriptor/problemDescriptor.h"

ProblemDescriptor::ProblemDescriptor() : m_name(""), m_gravity(0, 0, -9.81), m_mass(1) {}

ProblemDescriptor::~ProblemDescriptor()
{
  // std::cout << "ProblemDescriptor destructor called!" << '\n';
}

// ---------- Static function -----------

Eigen::Matrix3d ProblemDescriptor::skewSymmetric(Eigen::Vector3d const & vect)
{
  Eigen::Matrix3d vect_hat;
  vect_hat << 0, -vect(2), vect(1), vect(2), 0, -vect(0), -vect(1), vect(0), 0;
  return vect_hat;
}
