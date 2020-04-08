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

  The purpose of this class is to create a problem that defines a high dimensionnal convex that will then be projected to dimension 2 or 3 using the stabilityPolytope class. 

  The convex should decribed by equations of the form:
      A*x = b
      F*x <= f
 */

{
public:
  // ----------- constructors and destructor ----------
  
  ProblemDescriptor(std::string name="ProblemDescriptor");
  virtual ~ProblemDescriptor();

  // ----------- Main method of the class -------------
  /*! \brief return the Matrix A
   */
  inline const Eigen::MatrixXd & getMatrixA()
  {
    return m_A;
  }
  /*! \brief return the vector b
   */
  inline const Eigen::VectorXd & getVectorB()
  {
    return m_b;
  }
  /*! \brief return the Matrix F
   */
  inline const Eigen::MatrixXd & getFrictionF()
  {
    return m_F;
  }
  /*! \brief return the vector f
   */
  inline const Eigen::VectorXd & getFrictionVectorf()
  {
    return m_f;
  }

  /*! \brief recompute the matrices A and F and the vectors b and f
   */
  virtual void update() = 0;

  /*! \brief returns the name of the ProblemDescriptor
   */
  inline const std::string & get_name() const
  {
    return m_name;
  }

  /*! \brief set the name of the ProblemDescriptor
   */
  inline void set_name(std::string name)
  {
    m_name = name;
  }

protected:
  std::string m_name;
  // Eigen::Vector3d const m_gravity;
  // double m_mass;
  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_F;
  Eigen::VectorXd m_b;
  Eigen::VectorXd m_f;
};
