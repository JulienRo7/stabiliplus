#include "robot.h"

using namespace std;

Robot::Robot() : m_name(""), m_gravity(0,0,-9.81), m_mass(1), m_numberOfFeet(4), m_numberOfFrictionSides(8),
m_numberOfAccelerations(0)
{
    m_accelerations.push_back(m_gravity);
}

Robot::Robot(string const& robot_file_name, int numFrictionSides) :
m_gravity(0,0,-9.81), m_numberOfFrictionSides(numFrictionSides),
m_numberOfAccelerations(0)
{
    loadRobot(robot_file_name);
}

Robot::~Robot()
{
    // std::cout << "Robot destructor called!" << '\n';
}

Eigen::MatrixXd Robot::computeMatrixA1()
{
    int n_columns = 3*m_numberOfFeet;

    Eigen::MatrixXd A1(6,n_columns);

    for (int i(0); i<m_numberOfFeet; ++i)
    {
        A1.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
        A1.block<3,3>(3,3*i) = skewSymmetric(m_feet[i].get_position());
    }

    return A1;
}

Eigen::MatrixXd Robot::computeMatrixA2(Eigen::Vector3d const& acceleration)
{
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6,3);
    A2.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
    A2.block<3,3>(3,0) = -skewSymmetric(acceleration);

    return A2;
}

Eigen::VectorXd Robot::computeVector_t(Eigen::Vector3d const& acceleration)
{
    Eigen::VectorXd t(6);

    t.head(3) = -m_mass*acceleration;
    t.tail(3) = Eigen::Vector3d::Zero();

    return t;
}

Eigen::MatrixXd Robot::buildMatrixA()
{
    int const n_columnsA1 = 3*m_numberOfFeet;
    int const n_columnsA = 3*m_numberOfFeet*m_numberOfAccelerations + 3;
    int const n_rowsA = 6*m_numberOfAccelerations;

    Eigen::MatrixXd  A = Eigen::MatrixXd::Zero(n_rowsA, n_columnsA);
    Eigen::MatrixXd A1(6, n_columnsA1);

    A1 = computeMatrixA1();

    for (int i=0; i<m_numberOfAccelerations; ++i)
    {
        A.block(6*i, n_columnsA1*i, 6, n_columnsA1) = A1;
        A.block<6, 3>(6*i, n_columnsA-3) = computeMatrixA2(m_accelerations[i]);
    }

    return A;

}

Eigen::VectorXd Robot::buildVectorB()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(6*m_numberOfAccelerations);

    for (int i=0; i < m_numberOfAccelerations; ++i)
    {
        B.segment<6>(6*i) = computeVector_t(m_accelerations[i]);
    }
    return B;
}

Eigen::MatrixXd Robot::buildFrictionF()
{
    int const numberOfColumns = 3*m_numberOfFeet*m_numberOfAccelerations + 3;
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2)*m_numberOfAccelerations + 6;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(numberOfRows, numberOfColumns);
    Eigen::MatrixXd F_contact((m_numberOfFrictionSides+2), 3);

    for (int i=0; i<m_numberOfFeet; ++i)
    {
        F_contact = m_feet[i].linearizedFrictionCone(m_numberOfFrictionSides);

        for (int j=0; j<m_numberOfAccelerations; ++j)
        {
            F.block(j*(m_numberOfFrictionSides+2)*m_numberOfFeet + i*(m_numberOfFrictionSides+2), j*3*m_numberOfFeet + i*3 ,(m_numberOfFrictionSides+2), 3) = F_contact;
        }
    }

    F.block<3,3>(numberOfRows-6,numberOfColumns-3) = Eigen::Matrix3d::Identity();
    F.bottomRightCorner<3,3>() = -Eigen::Matrix3d::Identity();

    return F;
}

Eigen::VectorXd Robot::buildFrictionVectorf()
{
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2)*m_numberOfAccelerations + 6;
    Eigen::VectorXd f(numberOfRows);

    double f_max = 10*m_mass;

    for (int i=0; i<numberOfRows-6; i+=m_numberOfFrictionSides+2)
    {
        f[i] = f_max;
        f[i+1] = 0;
        for (int j=0; j<m_numberOfFrictionSides; ++j)
        {
            f[i+2+j]= 0.0;
        }
    }
    for (int i=numberOfRows-6; i<numberOfRows; i++)
    {
        f[i] = 1; // CoM position limited to the unit cube
    }

    return f;
}

// ----------- input functions ----------
void Robot::loadRobot(string const& file_name)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(file_name.c_str());

    if (doc.ErrorID() == 0)
    {

        tinyxml2::XMLElement* root = doc.FirstChildElement("data");
        tinyxml2::XMLElement* mainXML = root->FirstChildElement("robot");
        tinyxml2::XMLElement* childXML(0);
        std::string mainType;
        std::string childType;

        Eigen::Vector3d acceleration;
        tinyxml2::XMLElement* lineXML(0);


        while(mainXML)
        {
            mainType = mainXML->Value();
            if (std::strcmp(mainType.c_str(), "robot")==0)
            {
                m_name=mainXML->Attribute("name");
                childXML = mainXML->FirstChildElement();

                while (childXML) {
                    childType = childXML->Value();
                    if (std::strcmp(childType.c_str(), "Mass")==0)
                    {
                        childXML->QueryAttribute("mass", &m_mass);
                    }
                    else if (std::strcmp(childType.c_str(), "NumFeet")==0)
                    {
                        childXML->QueryAttribute("n_feet", &m_numberOfFeet);
                    }
                    else if (std::strcmp(childType.c_str(), "ContactPoint")==0)
                    {
                        m_feet.push_back(ContactPoints(childXML));
                        // cout << childXML->Attribute("name") << " added" << endl;
                    }
                    else {
                        std::cerr << "Un-Recognized child element name: " << childType << '\n';
                    }
                    childXML = childXML->NextSiblingElement();
                }

            }
            else if (std::strcmp(mainType.c_str(), "accelerations")==0)
            {
                childXML = mainXML->FirstChildElement();

                while (childXML)
                {
                    ++m_numberOfAccelerations;
                    // std::cout << "Add Acceleration!" << '\n';
                    childType = childXML->Value();
                    if (std::strcmp(childType.c_str(), "matrix")==0)
                    {
                        lineXML = childXML->FirstChildElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(0));
                        lineXML = lineXML->NextSiblingElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(1));
                        lineXML = lineXML->NextSiblingElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(2));

                        // std::cout << "New acceleration: " << '\n' << acceleration << '\n';

                        m_accelerations.push_back(acceleration);

                    }
                    else
                    {
                        std::cerr << "Un-Recognized child element name: " << childType << '\n';
                    }

                    childXML = childXML->NextSiblingElement();
                }
            }
            else
            {
                std::cerr << "Un-Recognized main element name: " << mainType << '\n';
            }

            mainXML = mainXML->NextSiblingElement();
        }

        // cout << robot_name << " loaded !" << endl;
    }
    else
    {
        cerr << "Failed to open the XML description file" << endl;
    }
}

// ----------- output and display functions ----------
void Robot::showRobot()
{
    cout << "Mass of the robot: " << m_mass << endl;
    cout << "Number of feet of the robot: " << m_numberOfFeet << endl;
}

void Robot::saveRobot(const std::string &file_name)
{
    // creating new xml object
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLDeclaration * declaration = doc.NewDeclaration();
    doc.InsertEndChild(declaration);

    // creating root node
    tinyxml2::XMLNode *root = doc.NewElement("data");
    doc.InsertEndChild(root);

    // building the robot element
    tinyxml2::XMLElement *XMLRobot = doc.NewElement("robot");
    XMLRobot->SetAttribute("name", m_name.c_str());

    {
        tinyxml2::XMLElement *XMLMass = doc.NewElement("Mass");
        XMLMass->SetAttribute("mass", m_mass);
        XMLRobot->InsertEndChild(XMLMass);

        tinyxml2::XMLElement *XMLNumFeet = doc.NewElement("NumFeet");
        XMLNumFeet->SetAttribute("n_feet", m_numberOfFeet);
        XMLRobot->InsertEndChild(XMLNumFeet);

        for (auto foot: m_feet)
        {
            XMLRobot->InsertEndChild(foot.get_XMLContactPoint(doc));
        }
    }
    root->InsertEndChild(XMLRobot);

    // building the accelerations element
    tinyxml2::XMLElement *XMLAccelerations = doc.NewElement("accelerations");
    std::string acc_name;

    for (int i=0; i<m_numberOfAccelerations; i++)
    {
        tinyxml2::XMLElement *XMLAcc = doc.NewElement("matrix");
        acc_name = "acceleration_"+std::to_string(i);
        XMLAcc->SetAttribute("name", acc_name.c_str());
        XMLAcc->SetAttribute("row", 3);
        XMLAcc->SetAttribute("column", 1);
        {
            for (int j=0; j<3; j++)
            {
                tinyxml2::XMLElement *XMLline = doc.NewElement("line");
                tinyxml2::XMLElement *XMLv = doc.NewElement("v");
                XMLv->SetText(m_accelerations[i](j));
                XMLline->InsertEndChild(XMLv);
                XMLAcc->InsertEndChild(XMLline);
            }
        }
        XMLAccelerations->InsertEndChild(XMLAcc);
    }

    root->InsertEndChild(XMLAccelerations);

    // saving the robot description xml file.
    doc.SaveFile(file_name.c_str());

}

// ------------------ Getters -----------------------

int Robot::get_numberOfFeet() const
{
    return m_numberOfFeet;
}

int Robot::get_numberOfAcceletations() const
{
    return m_numberOfAccelerations;
}



// ------------------ setter -----------------------
void Robot::translateContact(int contactIndex, Eigen::Vector3d translation)
{
    if (contactIndex >= 0 && contactIndex < m_feet.size())
    {
        m_feet[contactIndex].translate(translation);
    }
    else
    {
        std::cerr << "Error: the contact index is not valid" << '\n';
    }
}
// ---------- Static function -----------

Eigen::Matrix3d Robot::skewSymmetric(Eigen::Vector3d const& vect)
{
    Eigen::Matrix3d vect_hat;
    vect_hat << 0, -vect(2), vect(1),
                vect(2), 0, -vect(0),
                -vect(1), vect(0), 0;
    return vect_hat;
}
