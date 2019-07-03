#include "contactPoints.h"

// using namespace std;

ContactPoints::ContactPoints(): m_frictionCoef(1)
{
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            m_rotation(i,j)=0.0;
        }
        m_position(i) = 0.0;
    }
}

ContactPoints::ContactPoints(tinyxml2::XMLElement* contactPointXML)
{

    tinyxml2::XMLElement* currentXMLElement = contactPointXML->FirstChildElement();
    tinyxml2::XMLElement* lineXML(0);
    tinyxml2::XMLElement* valueXML(0);
    std::string currentType;


    while (currentXMLElement)
    {
        currentType = currentXMLElement->Name();
        if (std::strcmp(currentType.c_str(), "friction")==0)
        {
            // Get the friction coefficient of the contact Point
            currentXMLElement->QueryDoubleAttribute("mu", &m_frictionCoef);
        }
        else if (std::strcmp(currentType.c_str(), "matrix")==0)
        {
            // if the element if a matrix then it is either the rotation matrix or the position vector
            lineXML = currentXMLElement->FirstChildElement("line");
            if (std::strcmp(currentXMLElement->Attribute("name"),"rotation")==0)
            {
                // case of the rotation matrix
                for (int i=0; i<3; i++)
                {
                    valueXML = lineXML->FirstChildElement("v");
                    for (int j=0; j<3; j++)
                    {
                        valueXML->QueryDoubleText(&m_rotation(i,j));
                        valueXML = valueXML->NextSiblingElement("v");
                    }
                    lineXML = lineXML->NextSiblingElement("line");
                }
            }
            else if (std::strcmp(currentXMLElement->Attribute("name"),"position")==0)
            {
                // case of the position vector
                for (int i=0; i<3; i++)
                {
                    valueXML = lineXML->FirstChildElement("v");
                    valueXML->QueryDoubleText(&m_position(i));
                    lineXML = lineXML->NextSiblingElement("line");
                }
            }
            else
            {
                std::cerr << "Un-Recognized matrix name: " << currentXMLElement->Attribute("name") << '\n';
            }
        }
        else if (std::strcmp(currentType.c_str(), "rotation")==0)
        {
            // Get the friction coefficient of the contact Point
            double phi, theta, psi;
            currentXMLElement->QueryDoubleAttribute("phi", &phi);
            currentXMLElement->QueryDoubleAttribute("theta", &theta);
            currentXMLElement->QueryDoubleAttribute("psi", &psi);

            double const c1 = cos(phi);
            double const s1 = sin(phi);

            double const c2 = cos(theta);
            double const s2 = sin(theta);

            double const c3 = cos(psi);
            double const s3 = sin(psi);

            m_rotation(0, 0) = c2;
            m_rotation(0, 1) = -s2*c3;
            m_rotation(0, 2) = s2*s3;
            m_rotation(1, 0) = c1*s2;
            m_rotation(1, 1) = c1*c2*c3 - s1*s3;
            m_rotation(1, 2) = -c1*c2*s3 - s1*c3;
            m_rotation(2, 0) = s1*s2;
            m_rotation(2, 1) = s1*c2*c3 + c1*s3;
            m_rotation(2, 2) = -s1*c2*s3 + c1*c3;
        }
        else
        {
            std::cerr << "Un-Recognized Element name: " << currentType << '\n';
        }
        currentXMLElement = currentXMLElement->NextSiblingElement();
    }

    // showContactPoint();
    // std::cout << contactPointXML->Attribute("name") << " loaded!" << '\n';

}

ContactPoints::~ContactPoints()
{

}

void ContactPoints::showContactPoint()
{
    std::cout << "Contact Point Homogeneous Transform:" << '\n';
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            std::cout << m_rotation(i,j) << ' ';
        }
        std::cout << m_position(i) << '\n';
    }
    std::cout << "0 0 0 1" << '\n';
}

Eigen::MatrixXd ContactPoints::linearizedFrictionCone(int numberOfFrictionSides)
{
    Eigen::MatrixXd F(numberOfFrictionSides+2, 3);
    Eigen::RowVector3d line;

    line << 0, 0, 1;
    line = line*(m_rotation.transpose());
    F.row(0) = line;

    line << 0, 0, -1;
    line = line*(m_rotation.transpose());
    F.row(1) = line;


    double Dtheta(2*M_PI/numberOfFrictionSides);
    // std::cout << "Dtheta: " << Dtheta << '\n';
    for (int i=0; i<numberOfFrictionSides; ++i)
    {
        line << cos(i*Dtheta), sin(i*Dtheta), -m_frictionCoef*cos(Dtheta/2);
        // line << cos(i*Dtheta), sin(i*Dtheta), -m_frictionCoef;
        line = line*(m_rotation.transpose());
        F.row(i+2) = line;
    }

    return F;
}

Eigen::Vector3d ContactPoints::get_position()
{
    return m_position;
}

Eigen::Matrix3d ContactPoints::get_rotation()
{
    return m_rotation;
}
