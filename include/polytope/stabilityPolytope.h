#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#pragma once

// standard libraries
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <random>

// libraries
#include <Eigen/Dense>
#include <glpk.h> // to compute chebichev center
#include <tinyxml2.h>

// custom libraries
#include "wrapper/glpk_wrapper.h"
//#include "wrapper/lpsolve_wrapper.h"
//#include "wrapper/gurobi_wrapper.h"

//#include "problemDescriptor/contactSet.h"
#include "problemDescriptor/problemDescriptor.h"

/*!
 * \brief Abstract class for projectors
 *
 * This class is virtual.
 * Projectors compute the projection of high dimensionnal convex discribed using a `ProblemDescriptor` object and return a 2D or 3D convex using the convex projection algorithm.
 *
 */

class StabilityPolytope
{
public:
  // ----------- constructors and destructor ----------

  /*!
   * \brief class constructor
   *
   * \param inputPD Object describing the convex to project
   * \param maxIteration Stop criterion for the algorithm
   * \param maxError Stop criterion for the algorithm
   * \param solveType Name of the LP solver used
   */
  StabilityPolytope(std::shared_ptr<ProblemDescriptor> inputPD, int maxIteration, double maxError, Solver solveType);

  /*!
   * \brief virtual destructor
   */
  virtual ~StabilityPolytope();

  // ----------- main class methods ----------
  /*!
   * \brief Initialized the solver with the problem described by the ProblemDescriptor
   */
  virtual void initSolver() = 0;
  
  // virtual void solveLP(Eigen::Vector3d const& direction, Eigen::Vector3d &vertex) = 0;

  /*!
   * \brief Main function to project the convex
   */
  virtual void projectionStabilityPolyhedron() = 0;

  /*!
   * \brief free the solver object 
   * Hopefully usefull when having several instances of stabilitiy polytope
   */ 
  virtual void endSolver();
    
  /*!
   * \brief Check if the stopCriterion as been reached
   */
  bool stopCriterion() const; // return true when the algorithm must stop

  // ----------- output and display functions ----------
  /*!
   * \brief write the output of the projection algorithm to stream using an homemade algorithm
   * 
   */
  virtual void writeToStream(std::ofstream & stream) const = 0;

  /*!
   * \brief build the xml object describing the polytope
   */
  virtual tinyxml2::XMLElement * xmlPolytope(tinyxml2::XMLDocument & doc) const;

  /*! \brief save the polytope in xml format
   */
  void saveToFile(std::string fileName) const;
  
  /*!
   * \brief return the H-Rep of the projected convex
   * 
   * `Aineq` represents half-planes and `bineq` represents offsets.
   *
   */
  virtual void computeHrep(Eigen::MatrixXd & Aineq, Eigen::VectorXd & bineq) const = 0;
  
  /*!
   * \brief return the H-Rep of the projected convex
   * 
   * Each `Eigen::Vector4d` represents one hyperplane of the projected convex. 
   * The first 3 coordinates represent the normal of the hyperplane toward the outside of the polytope
   * The 4th coodinate is the offset.
   *
   */
  virtual std::vector<Eigen::Vector4d> constraintPlanes() const = 0;

  /*! \brief return the V-Rep of the projected convex
   * 
   */
  virtual std::vector<Eigen::Vector3d> vertices() const = 0;
  
  /*!
   * \brief Return a point inside the convex polytope: the average sum on all the vertices
   *
   */
  virtual Eigen::Vector3d baryPoint() const = 0;

  // ----------- getters ----------

  inline const int getIteration(){
    return m_iteration;
  }
  inline const double getError(){
    return m_error;
  }

  /*!
   * \brief returns total time spent computing LPs
   */
  double LPTime() const;

  /*!
   * \brief returns time spent initializing the problem
   */ 
  double initTime() const;

  /*!
   * \brief returns time spent conputing the inner and outer approximation of the convex
   */
  double structTime() const;

  /*!
   * \brief returns pointer to `ProblemDescriptor`
   */
  virtual inline std::shared_ptr<ProblemDescriptor> problemDescriptor()
  {
    return m_pdPtr;
  }
  
  Solver solverType() const;

  virtual int get_numberOfVertices() const = 0;

  inline int getMaxIteration() const
  {
    return m_maxIteration; 
  }

  inline double getMaxError() const
  {
    return m_maxError;
  }

  inline int getIteration() const
  {
    return m_iteration;
  }

  inline int getErrorCode() const
  {
    return errorCode_;
  }

  // ----------- setters ----------
  void maxIteration(int maxIteration);

  // ---------- static functions ---------

  /* \brief Compute the Chebichev center of a set of planes
   * Chebichev Center is the center of the biggest ball inscribed in the convex set defined by the set of planes
   * It can also be called the deeppest point in the convex
   * Here the computation is done only in dimension 3.
   */
  Eigen::Vector3d chebichevCenter(std::vector<Eigen::Vector4d> planes) const;
  virtual Eigen::Vector3d chebichevCenter() const;

protected:
  // robot:
  std::shared_ptr<ProblemDescriptor> m_pdPtr; // for now the contact set should not change

  // attributes used for the LP problem
  Solver m_solverType;
  SolverWrapper * m_lp;
  bool m_solverEnded;

  // projection algorithm stop criterion
  int m_iteration;
  double m_error;

  // options
  int m_maxIteration;
  double m_maxError;

  // time measures
  double m_LPTime;
  double m_initTime;
  double m_structTime;

  // errors
  int errorCode_=-1;

}; // class StabilityPolytope
