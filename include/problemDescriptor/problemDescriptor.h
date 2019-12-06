#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#pragma once

/*
The robot class contains the description of the robot. It can load it and make other things
*/

// standart libraries
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// libraries
#include <Eigen/Dense>
#include <tinyxml2.h>

class ProblemDescriptor
{
public:
  // ----------- constructors and destructor ----------
  ProblemDescriptor();
  virtual ~ProblemDescriptor();

  virtual Eigen::MatrixXd buildStaticMatrixA() = 0;
  virtual Eigen::VectorXd buildStaticVectorB() = 0;
  virtual Eigen::MatrixXd buildStaticFrictionF() = 0;
  virtual Eigen::VectorXd buildStaticFrictionVectorf() = 0;

  virtual Eigen::MatrixXd buildMatrixA() = 0;
  virtual Eigen::VectorXd buildVectorB() = 0;
  virtual Eigen::MatrixXd buildFrictionF() = 0;
  virtual Eigen::VectorXd buildFrictionVectorf() = 0;

  inline const std::string & get_name() const
  {
    return m_name;
  }

  // ---------- static functions ---------
  static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const & vect);

protected:
  std::string m_name;
  Eigen::Vector3d const m_gravity;
  double m_mass;
};
