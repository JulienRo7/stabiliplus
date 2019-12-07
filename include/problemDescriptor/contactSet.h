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

// custom libraries
#include "problemDescriptor/contactPoints.h"
#include "problemDescriptor/problemDescriptor.h"

class ContactSet : public ProblemDescriptor
{

public:
  // ----------- constructors and destructor ----------
  ContactSet(bool staticCase);
  ContactSet(bool staticCase, std::string const & robot_file_name, int numFrictionSides = 8);
  ~ContactSet();

  // ----------- main class methods ----------
  void update() override;

  // ----------- input functions ----------
  void loadContactSet(std::string const & file_name);

  // ----------- output and display functions ----------
  void showContactSet();
  void saveContactSet(const std::string & file_name);

  // ----------- getters ----------
  int get_numberOfFeet() const;

  int get_numberOfAcceletations() const;

  int get_contactIndexFromName(std::string contactName) const;
  std::vector<std::string> get_contactNames() const;

  bool hasContactNamed(std::string contactName) const;

  // ----------- setters ----------
  void translateContact(int contactIndex, Eigen::Vector3d translation);
  void updateContact(int contactIndex, Eigen::Matrix4d homTrans);
  void updateContact(std::string contactName, Eigen::Matrix4d homTrans);

  void removeContact(int contactIndex);
  void removeContact(std::string contactName);

  void addContact(std::string contactName);
  void addContact(std::string contactName, Eigen::Matrix4d homTrans, double friction = 0.5);

  inline bool staticCase() const
  {
    return staticCase_;
  }
void setStaticCase(bool setTrue) 
  {
    staticCase_ = setTrue; 
  }

private:
  bool staticCase_;
  // std::string m_name;
  // Eigen::Vector3d const m_gravity;
  // double m_mass;

  int m_numberOfFeet = 0;
  int m_numberOfFeet_ini = -1;
  std::vector<ContactPoints> m_feet;

  int m_numberOfAccelerations = 0;
  int m_numberOfAccelerations_ini = -1;
  std::vector<Eigen::Vector3d> m_accelerations;

  /*! \brief Check whether needs to update the matrix size depending on the number of feet and accelerations.
   */
  bool needsUpdateSize_();
  // bool needsUpdateStaticSize_();
  void resetMatricies_();
  void resetStaticMatricies_();
  void setZeroMatricies_();
  int m_numberOfFrictionSides; // Number of sides of the approximation of the friction cones

  // Matrix constructors
  void buildStaticMatrixA_();
  void buildStaticVectorB_();
  void buildStaticFrictionF_();
  void buildStaticFrictionVectorf_();

  void buildMatrixA_();
  void computeMatrixA1_(Eigen::MatrixXd & A1);
  void computeMatrixA2_(Eigen::MatrixXd & A2, Eigen::Vector3d const & acceleration);

  void buildVectorB_();
  void buildFrictionF_();
  void buildFrictionVectorf_();
  Eigen::VectorXd computeVector_t_(Eigen::Vector3d const & acceleration);
};
