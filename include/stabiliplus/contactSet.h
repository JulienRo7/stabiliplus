#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef CONTACTSET_H_INCLUDED
#define CONTACTSET_H_INCLUDED

/*
The robot class contains the description of the robot. It can load it and make other things
*/
// standart libraries
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include <chrono>
#include <memory>

// libraries
#include <tinyxml2.h>
#include <Eigen/Dense>

// custom libraries
#include "contactPoints.h"


class ContactSet
{

public:

    // ----------- constructors and destructor ----------
    ContactSet();
    ContactSet(std::string const& robot_file_name, int numFrictionSides=8);
    ~ContactSet();

    // ----------- main class methods ----------
    Eigen::MatrixXd computeMatrixA1();
    Eigen::MatrixXd computeMatrixA2(Eigen::Vector3d const& acceleration);
    Eigen::VectorXd computeVector_t(Eigen::Vector3d const& acceleration);

    Eigen::MatrixXd buildMatrixA();
    Eigen::VectorXd buildVectorB();
    Eigen::MatrixXd buildFrictionF();
    Eigen::VectorXd buildFrictionVectorf();

    // ----------- input functions ----------
    void loadContactSet(std::string const& file_name);

    // ----------- output and display functions ----------
    void showContactSet();
    void saveContactSet(const std::string &file_name);


    // ----------- getters ----------
    int get_numberOfFeet() const;
    int get_numberOfAcceletations() const;

    int get_contactIndexFromName(std::string contactName) const;
    std::vector<std::string> get_contactNames() const;

    bool hasContactNamed(std::string contactName) const;
    
    std::string get_name() const;

    // ----------- setters ----------
    void translateContact(int contactIndex, Eigen::Vector3d translation);
    void set_contact(int contactIndex, Eigen::Matrix4d homTrans);

    void removeContact(int contactIndex);
    void removeContact(std::string contactName);

    void addContact(std::string contactName);

    // ---------- static functions ---------
    static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const& vect);

private:
    std::string m_name;
    Eigen::Vector3d const m_gravity;
    double m_mass;

    int m_numberOfFeet;
    std::vector<ContactPoints> m_feet;

    int m_numberOfAccelerations;
    std::vector<Eigen::Vector3d> m_accelerations;

    int m_numberOfFrictionSides; // Number of sides of the approximation of the friction cones

};

#endif // CONTACTSET_H_INCLUDED
