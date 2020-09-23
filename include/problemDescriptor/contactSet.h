#ifndef _USE_MATH_DEFINES
#  define _USE_MATH_DEFINES
#endif

#pragma once

/*!
The contactSet class contains the description of the contactSet. It can load it and make other things
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

// custom libraries
#include "problemDescriptor/contactPoints.h"
#include "problemDescriptor/problemDescriptor.h"

class ContactSet : public ProblemDescriptor
{

public:
  // ----------- constructors and destructor ----------
  ContactSet(bool staticCase);
  ContactSet(bool staticCase, std::string const & robot_file_name, int numFrictionSides = 8);

  /*! \brief Copy constructor
   */
  //ContactSet(const ContactSet& other);
  
  /*! \brief destructor
   */
  ~ContactSet();

  // ----------- main class methods ----------
  /*! Update the contactSet, changes the matrices size if needed, reset them and recompute them
   */
  void update();

  /*! Check whether needs to update the matrix size depending on the number of feet and accelerations.
   */
  bool checkMatricesSizes() const;
  /*!
    Recompute the matrices size and resize them accordingly
   */
  void updateMatricesSizes();
  /*!
    Set all matrices to 0
  */
  void setZeroMatrices();


  /*! \brief Build the matrix A
    The A matrix contains the sum of the contact forces, the gravity momentum and the inertial momemtum
    Stacks together matrices of type A1 and A2 depending on the number of accelerations
  */
  void buildMatrixA();
  
  /*! \brief Compute matrix of type A1
    Matrix of type A1 represent the sum of contact forces and the sum of generated moment in the Newton-Euler equations.
  */
  void computeMatrixA1(Eigen::MatrixXd & A1);
  
  /*! \brief Compute matrix of type A2
    Matrix of type A2 represent the gravity/inertial momentum in the Newton-Euler equations
   */
  void computeMatrixA2(Eigen::MatrixXd & A2, Eigen::Vector3d const & acceleration);

  /*! \brief compute the left-hand part of the equality constraint
    When there are several accelerations it stacks vectors t overwise it is equals to t
    \sa contactSet::computeVectort
   */
  void buildVectorB();
  
  /*! \brief compute the sum of known force in the Newton Euler equations
   */
  void computeVectort(Eigen::VectorXd & t, Eigen::Vector3d const & acceleration);

  /*! \brief Builds the left hand side of the inequality constraint
    Build the friction matrix and add geometric limits to the CoM position 
   */
  void buildFrictionF();

  /*! \brief Build the right hand side of the inequality constraint
   */
  void buildFrictionVectorf();
  
  // ----------- input functions ----------
  void loadContactSet(std::string const & file_name);

  // ----------- output and display functions ----------
  void showContactSet();
  void saveContactSet(const std::string & file_name);

  // ----------- getters ----------
  inline const int numberOfContacts() const
  {
    return m_contacts.size();
  }

  inline const int get_numberOfAccelerations() const
  {
    return m_accelerations.size();
  }

  /*! \brief given a contact Name returns its index
   *
   */
  int get_contactIndexFromName(std::string contactName) const;
  
  /*! \brief List of the contact points names in the contact set object 
   *
   */
  std::vector<std::string> get_contactNames() const;

  /*! \brief Check if the contact Set has a contact with the given name
   *
   */
  bool hasContactNamed(std::string contactName) const;

  /*! \brief accessor to the maximal normal force of a contact 
   */
  double contactFMax(std::string contactName) const;

  /*! \brief accessor to the minimal normal force of a contact 
   */
  double contactFMin(std::string contactName) const;

  /*! \brief accessor to the mass of the robot with the current contact Set
   */
  inline const double mass() const
  {
    return m_mass;
  }

  /*! \brief returns position and orientatio of a contact given its name
   */
  Eigen::Matrix4d contactHomTrans(std::string contactName) const;

  /*! \brief check if the contact set has a constrained contact
   */ 
  bool hasConstrainedContact() const;
  
  /*! \brief return the list of constrained contact
   */
  std::vector<std::string> constrainedContactNames() const;

  /*! \brief return the number on constrained contacts
   */
  int numberConstrainedContacts() const;
  
  // ----------- setters ----------
  void addContact(std::string contactName);
  void addContact(std::string contactName, Eigen::Matrix4d homTrans, double friction = 0.5, double fmax=1000, double fmin=0, ContactType type=ContactType::support);
  
  void updateContact(int contactIndex, Eigen::Matrix4d homTrans);
  void updateContact(std::string contactName, Eigen::Matrix4d homTrans);

  /*!\brief Translate a contact position
   * @param contactIndex index of the contact
   * @param translation applied to the contact 
   */
  void translateContact(int contactIndex, Eigen::Vector3d translation);
  /*!\brief Translate a contact position
   * @param contactName name of the contact
   * @param translation applied to the contact 
   */
  void translateContact(std::string contactName, Eigen::Vector3d translation);
  
  void removeContact(int contactIndex);
  void removeContact(std::string contactName);

  /*! \brief set the contact maximal normal force
   */
  void setContactFMax(double fmax, std::string contactName);

  /*! \brief set the contact minimal normal force
   */
  void setContactFMin(double fmin, std::string contactName);
  
  inline bool staticCase() const
  {
    return staticCase_;
  }
  void setStaticCase(bool setTrue)
  {
    staticCase_ = setTrue;
  }

  /*! \brief set the mass of the rbot associated with the current contact Set
   *  checks if the mass is 0 or lower that 0 
   */
  void mass(double mass);

  /*! \brief Method to add an acceleration to the problem
   */
  void addCoMAcc(Eigen::Vector3d acceleration);

  /*!
   */
  inline void setFrictionSides(int fricSides)
  {
    m_numberOfFrictionSides = fricSides;
  }

  inline int frictionSides() const
  {
    return m_numberOfFrictionSides;
  }

  /*! \brief setter to change the type of a contact 
   * The valid values for the type are support or constrained
   */ 
  void updateContactType(std::string contactName, ContactType type);

  void printAcc();

  inline const int globRows() const
  {
    return m_globRows;
  }
  
  inline const int globCols() const
  {
    return m_globCols;
  }

  Eigen::VectorXd Ydes() const;
  Eigen::MatrixXd forcePos() const;
  
  // ---------- static functions ---------
  static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const & vect);
  
private:

  //Eigen::Vector3d const m_gravity;
  /*
   * Having a coherent mass is important because for this contact set there are force limits on the contact: If the mass is too important then one contact point may not be strong enough to withstand the robot alone. This add some restrictions on the equilibrium region.
   */
  double m_mass;

  /*!
   * Dimension of the projected result: 
   *     - static case: 2
   *     - dynamic/robust case: 3
   */
  int m_dim;
  
  std::vector<ContactPoints> m_contacts;
  std::vector<Eigen::Vector3d> m_accelerations;

  /*! \brief Number of columns in one subproblem 
    \note Each subproblem correspond to one acceleration, in the static case there should be ony one acceleration: gravity.
   */
  int m_subCols;
  /*! \brief Number of optimization variables 
   */
  int m_globCols;
  
  /*! \brief Number of sides for the approximation of the friction cones
   */
  int m_numberOfFrictionSides;

  /*! \brief Number of rows in one subproblem 
    \note Each subproblem correspond to one acceleration, in the static case there should be ony one acceleration: gravity.
   */
  int m_subRows;

  /*! \brief Nomber of inquality constraints in the problem
   */
  int m_globRows;
  
  bool staticCase_;
};
