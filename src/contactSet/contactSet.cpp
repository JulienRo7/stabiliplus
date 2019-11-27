#include "contactSet/contactSet.h"

// using namespace std;

ContactSet::ContactSet() : m_name(""), m_gravity(0,0,-9.81), m_mass(1), m_numberOfFeet(0), m_numberOfFrictionSides(8),
m_numberOfAccelerations(1)
{
    m_accelerations.push_back(m_gravity);
}

ContactSet::ContactSet(std::string const& contact_set_file_name, int numFrictionSides) :
m_gravity(0,0,-9.81), m_numberOfFrictionSides(numFrictionSides),
m_numberOfAccelerations(0)
{
  loadContactSet(contact_set_file_name);
}

ContactSet::~ContactSet()
{
    // std::cout << "ContactSet destructor called!" << '\n';
}

Eigen::MatrixXd ContactSet::computeMatrixA1()
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

Eigen::MatrixXd ContactSet::computeMatrixA2(Eigen::Vector3d const& acceleration)
{
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6,3);
    A2.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
    A2.block<3,3>(3,0) = -skewSymmetric(m_mass*acceleration);

    return A2;
}

Eigen::VectorXd ContactSet::computeVector_t(Eigen::Vector3d const& acceleration)
{
    Eigen::VectorXd t(6);

    t.head(3) = -m_mass*acceleration;
    t.tail(3) = Eigen::Vector3d::Zero();

    return t;
}

Eigen::MatrixXd ContactSet::buildMatrixA()
{

  int const n_columnsA1 = 3*m_numberOfFeet;
  int const n_columnsA = 3*m_numberOfFeet*m_numberOfAccelerations + 3;
  int const n_rowsA = 6*m_numberOfAccelerations;

  Eigen::MatrixXd  A = Eigen::MatrixXd::Zero(n_rowsA, n_columnsA);
    //Eigen::MatrixXd A1(6, n_columnsA1);

  auto A1 = computeMatrixA1();


  for (int i=0; i<m_numberOfAccelerations; ++i)
    {
      A.block(6*i, n_columnsA1*i, 6, n_columnsA1) = A1;
      A.block<6, 3>(6*i, n_columnsA-3) = computeMatrixA2(m_accelerations[i]);
    }

  return A;
}

Eigen::MatrixXd ContactSet::buildStaticMatrixA()
{
  int const n_colA1 = 3*m_numberOfFeet;
  int const n_colA = n_colA1 + 2;
  int const n_rowA = 6;

  Eigen::MatrixXd A(n_rowA, n_colA);
  auto A1 = computeMatrixA1();

  A.leftCols(n_colA1) = A1;
  A.rightCols(2) = computeMatrixA2(m_gravity).leftCols(2);

  return A;

}

Eigen::VectorXd ContactSet::buildVectorB()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(6*m_numberOfAccelerations);

    for (int i=0; i < m_numberOfAccelerations; ++i)
    {
	B.segment<6>(6*i) = computeVector_t(m_accelerations[i]);
    }
    return B;
}

Eigen::VectorXd ContactSet::buildStaticVectorB()
{
  return computeVector_t(m_gravity);
}

Eigen::MatrixXd ContactSet::buildFrictionF()
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

Eigen::MatrixXd ContactSet::buildStaticFrictionF()
{
    int const numberOfColumns = 3*m_numberOfFeet + 2;
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2) + 4;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(numberOfRows, numberOfColumns);
    Eigen::MatrixXd F_contact((m_numberOfFrictionSides+2), 3);

    for (int i=0; i<m_numberOfFeet; ++i)
    {
	F_contact = m_feet[i].linearizedFrictionCone(m_numberOfFrictionSides);

	F.block(i*(m_numberOfFrictionSides+2), i*3 , m_numberOfFrictionSides+2, 3) = F_contact;
    }

    F.block<2,2>(numberOfRows-4,numberOfColumns-2) = Eigen::Matrix2d::Identity();
    F.bottomRightCorner<2,2>() = -Eigen::Matrix2d::Identity();

    return F;
}

Eigen::VectorXd ContactSet::buildFrictionVectorf()
{
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2)*m_numberOfAccelerations + 6;
    Eigen::VectorXd f(numberOfRows);
    f = Eigen::VectorXd::Zero(numberOfRows);

    // double f_max = 10*m_mass;

    for (int i=0; i<m_numberOfFeet; ++i)
      {
	int ind = i*(m_numberOfFrictionSides+2)*m_numberOfAccelerations;

	f[ind] = m_feet[i].fmax();
	f[ind+1] = -m_feet[i].fmin();
	// std::cout << "Setting inequalities for " << m_feet[i].get_name() << " fmax=" << m_feet[i].fmax() << "N fmin=" << m_feet[i].fmin() << "N" << std::endl;
      }

    // Limitation of the CoM
    f[numberOfRows-6] = 1000; // x_max
    f[numberOfRows-5] = 1000; // y_max
    f[numberOfRows-4] = 2; // z_max
    f[numberOfRows-3] = 1000; // -x_min
    f[numberOfRows-2] = 1000; // -y_min
    f[numberOfRows-1] = 0; // -z_min

    return f;
}

Eigen::VectorXd ContactSet::buildStaticFrictionVectorf()
{
  int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2) + 4;
  Eigen::VectorXd f(numberOfRows);
  f = Eigen::VectorXd::Zero(numberOfRows);

  // double f_max = 10*m_mass;

  for (int i=0; i<m_numberOfFeet; i+=1)
    {
      int init = i*(m_numberOfFrictionSides+2);
      f[init] = m_feet[i].fmax();
      f[init+1] = -m_feet[i].fmin();
    }

  // Limitation of the CoM
  f[numberOfRows-4] = 1000; // x_max
  f[numberOfRows-3] = 1000; // y_max
  f[numberOfRows-2] = 1000; // -x_min
  f[numberOfRows-1] = 1000; // -y_min

  return f;
}

// ----------- input functions ----------
void ContactSet::loadContactSet(std::string const& file_name)
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
	    if (std::strcmp(mainType.c_str(), "robot")==0) // FIX ME: for now in the XML files the contact set is still named robot
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

	// cout << robot_name << " loaded !" << std::endl;
    }
    else
    {
      std::cerr << "Failed to open the XML description file" << std::endl;
    }
}

// ----------- output and display functions ----------
void ContactSet::showContactSet()
{
  std::cout << "Contact Set name: " << m_name << std::endl;
  std::cout << "Mass of the robot: " << m_mass << std::endl;
  std::cout << "Number of feet of the robot: " << m_numberOfFeet << std::endl;

  for (auto contact : m_feet)
    {
      std::cout << "Contact named: " << contact.get_name() << " (fmax=" << contact.fmax() << "N, fmin=" << contact.fmin() << "N)" << std::endl;
    }

}

void ContactSet::saveContactSet(const std::string &file_name)
{
    // creating new xml object
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLDeclaration * declaration = doc.NewDeclaration();
    doc.InsertEndChild(declaration);

    // creating root node
    tinyxml2::XMLNode *root = doc.NewElement("data");
    doc.InsertEndChild(root);

    // building the robot element
    tinyxml2::XMLElement *XMLContactSet = doc.NewElement("robot");
    XMLContactSet->SetAttribute("name", m_name.c_str());

    {
	tinyxml2::XMLElement *XMLMass = doc.NewElement("Mass");
	XMLMass->SetAttribute("mass", m_mass);
	XMLContactSet->InsertEndChild(XMLMass);

	tinyxml2::XMLElement *XMLNumFeet = doc.NewElement("NumFeet");
	XMLNumFeet->SetAttribute("n_feet", m_numberOfFeet);
	XMLContactSet->InsertEndChild(XMLNumFeet);

	for (auto foot: m_feet)
	{
	    XMLContactSet->InsertEndChild(foot.get_XMLContactPoint(doc));
	}
    }
    root->InsertEndChild(XMLContactSet);

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

int ContactSet::get_numberOfFeet() const
{
    return m_numberOfFeet;
}

int ContactSet::get_numberOfAcceletations() const
{
    return m_numberOfAccelerations;
}

int ContactSet::get_contactIndexFromName(std::string contactName) const
{
    int i(0), index(0);
    for (auto contact: m_feet)
    {
	if (contact.get_name()==contactName)
	{
	    index=i;
	}
	i ++;
    }
    return index;
}

std::vector<std::string> ContactSet::get_contactNames() const
{
    std::vector<std::string> names;
    for (auto contact : m_feet)
    {
	names.push_back(contact.get_name());
    }
    return names;
}

bool ContactSet::hasContactNamed(std::string contactName) const
{
    std::vector<std::string> names = get_contactNames();
    return find(names.begin(), names.end(), contactName)!=names.end();
}

std::string ContactSet::get_name() const
{
    return m_name;
}


// ------------------ setter -----------------------
void ContactSet::translateContact(int contactIndex, Eigen::Vector3d translation)
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

void ContactSet::updateContact(int contactIndex, Eigen::Matrix4d homTrans)
{
    if (contactIndex >= 0 && contactIndex < m_feet.size())
    {
	m_feet[contactIndex].set_contact(homTrans);
    }
    else
    {
	std::cerr << "Error: the contact index is not valid" << '\n';
    }
}

void ContactSet::updateContact(std::string contactName, Eigen::Matrix4d homTrans)
{
  if (hasContactNamed(contactName))
    {
      int index = get_contactIndexFromName(contactName);
      m_feet[index].set_contact(homTrans);
    }
  else
    {
      std::cout << "There is no contact named " << contactName << "..." << std::endl;
      std::cout << "But I won't throw an error! hehehe!" << std::endl;
    }
}

void ContactSet::removeContact(int contactIndex)
{
    m_feet.erase(m_feet.begin()+contactIndex);
    m_numberOfFeet --;
}

void ContactSet::removeContact(std::string contactName)
{
    int index = get_contactIndexFromName(contactName);
    removeContact(index);
}

void ContactSet::addContact(std::string contactName)
{
    m_feet.push_back(ContactPoints(contactName, 0.5));
    m_numberOfFeet ++;
}

void ContactSet::addContact(std::string contactName, Eigen::Matrix4d homTrans, double friction)
{
  ContactPoints contact(contactName, friction);
  contact.set_contact(homTrans);
  m_feet.push_back(contact);
  m_numberOfFeet ++;
}

// ---------- Static function -----------

Eigen::Matrix3d ContactSet::skewSymmetric(Eigen::Vector3d const& vect)
{
    Eigen::Matrix3d vect_hat;
    vect_hat << 0, -vect(2), vect(1),
		vect(2), 0, -vect(0),
		-vect(1), vect(0), 0;
    return vect_hat;
}
