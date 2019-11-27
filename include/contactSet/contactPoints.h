#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef CONTACTPOINT_H_INCLUDED
#define CONTACTPOINT_H_INCLUDED

#include <iostream>
#include <string>
#include "math.h"

#include "tinyxml2.h"
#include <Eigen/Dense>

class ContactPoints
{
public:
    // ------- constructors and destructor -------
    ContactPoints();
    ContactPoints(std::string name, double frictionCoef, double fmax=1000, double fmin=0);
    ContactPoints(tinyxml2::XMLElement* contactPointXML);
    ~ContactPoints();

    // ------- class' main methods -------
    void showContactPoint();
    Eigen::MatrixXd linearizedFrictionCone(int numberOfFrictionSides);

    // ------- getters -------
    std::string get_name() const;
    Eigen::Vector3d get_position() const;
    Eigen::Matrix3d get_rotation() const;
    Eigen::Matrix4d get_homTrans() const;
    tinyxml2::XMLElement* get_XMLContactPoint(tinyxml2::XMLDocument &doc) const;

    bool isContactNamed(std::string name) const;

    double fmax() const;
    double fmin() const;

    // ------- setters -------
    void translate(Eigen::Vector3d trans);
    void set_contact(Eigen::Matrix4d homTrans);
    void set_friction(double frictionCoef);

    void fmax(double f);
    void fmin(double f);

private:
    std::string m_name;
    double m_frictionCoef;
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_position;

    double fmax_;
    double fmin_;

};

#endif // CONTACTPOINT_H_INCLUDED
