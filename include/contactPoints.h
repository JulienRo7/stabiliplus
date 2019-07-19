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
    ContactPoints();
    ContactPoints(tinyxml2::XMLElement* contactPointXML);
    ~ContactPoints();

    void showContactPoint();
    Eigen::MatrixXd linearizedFrictionCone(int numberOfFrictionSides);

    std::string get_name() const;
    Eigen::Vector3d get_position() const;
    Eigen::Matrix3d get_rotation() const;

    void translate(Eigen::Vector3d trans);

private:
    std::string m_name;
    double m_frictionCoef;
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_position;


};

#endif // CONTACTPOINT_H_INCLUDED
