#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#pragma once

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
/*! \brief It describes the polytope with inequalities and the projection with equalities.
 */

{
public:
  // ----------- constructors and destructor ----------
  ProblemDescriptor();
  virtual ~ProblemDescriptor();
  /*
    virtual Eigen::MatrixXd buildStaticMatrixA() = 0;
    virtual Eigen::VectorXd buildStaticVectorB() = 0;
    virtual Eigen::MatrixXd buildStaticFrictionF() = 0;
    virtual Eigen::VectorXd buildStaticFrictionVectorf() = 0;

    virtual Eigen::MatrixXd buildMatrixA() = 0;
    virtual Eigen::VectorXd buildVectorB() = 0;
    virtual Eigen::MatrixXd buildFrictionF() = 0;
    virtual Eigen::VectorXd buildFrictionVectorf() = 0;
  */

  /*
  const Eigen::MatrixXd &  getStaticMatrixA() = 0;
  const Eigen::VectorXd &  getStaticVectorB() = 0;
  const Eigen::MatrixXd &  buildStaticFrictionF() = 0;
  const Eigen::VectorXd &  buildStaticFrictionVectorf() = 0;
  */
  inline const Eigen::MatrixXd & getMatrixA()
  {
    return m_A;
  }

  inline const Eigen::VectorXd & getVectorB()
  {
    return m_B;
  }
  inline const Eigen::MatrixXd & getFrictionF()
  {
    return m_F;
  }
  inline const Eigen::VectorXd & getFrictionVectorf()
  {
    return m_f;
  }

  virtual void update() = 0;

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
  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_F;
  Eigen::VectorXd m_B;
  Eigen::VectorXd m_f;
};
