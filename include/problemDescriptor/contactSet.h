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
  ContactSet();
  ContactSet(std::string const & robot_file_name, int numFrictionSides = 8);
  ~ContactSet();

  // ----------- main class methods ----------
  Eigen::MatrixXd buildStaticMatrixA() override;
  Eigen::VectorXd buildStaticVectorB() override;
  Eigen::MatrixXd buildStaticFrictionF() override;
  Eigen::VectorXd buildStaticFrictionVectorf() override;

  Eigen::MatrixXd buildMatrixA() override;
  Eigen::VectorXd buildVectorB() override;
  Eigen::MatrixXd buildFrictionF() override;
  Eigen::VectorXd buildFrictionVectorf() override;
  Eigen::MatrixXd computeMatrixA1();
  Eigen::MatrixXd computeMatrixA2(Eigen::Vector3d const & acceleration);
  Eigen::VectorXd computeVector_t(Eigen::Vector3d const & acceleration);

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

private:
  // std::string m_name;
  // Eigen::Vector3d const m_gravity;
  // double m_mass;

  int m_numberOfFeet;
  std::vector<ContactPoints> m_feet;

  int m_numberOfAccelerations;
  std::vector<Eigen::Vector3d> m_accelerations;

  int m_numberOfFrictionSides; // Number of sides of the approximation of the friction cones
};
