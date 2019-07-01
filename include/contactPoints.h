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

    Eigen::Vector3d get_position();
    Eigen::Matrix3d get_rotation();

private:
    double m_frictionCoef;
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_position;


};

#endif // CONTACTPOINT_H_INCLUDED
