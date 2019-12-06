#include "wrapper/solver_wrapper.h"

SolverWrapper::SolverWrapper()
: m_result(Eigen::Vector3d::Zero()), m_searchDirection(Eigen::Vector3d::Zero()), m_problemSolved(true)
{
}

SolverWrapper::~SolverWrapper() {}

Eigen::Vector3d SolverWrapper::get_result()
{
  return m_result;
}

Eigen::Vector2d SolverWrapper::get_staticResult()
{
  return m_result.tail(2);
}
