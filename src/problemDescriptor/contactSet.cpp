#include "problemDescriptor/contactSet.h"

// using namespace std;

ContactSet::ContactSet(bool staticCase): ProblemDescriptor("ContactSet_1"),
					 m_numberOfFrictionSides(8),
					 staticCase_(staticCase),
					 m_mass(1)
{
  if (staticCase_)
    {
      m_dim = 2;
      // Eigen::Vector3d gravity;
      // gravity << 0, 0, -9.81;
      // m_accelerations.push_back(gravity);
    }
  else
    {
      m_dim = 3;
    }
  

  // std::cout << "The first constructor for contactSet has been called!" << std::endl;
}

ContactSet::ContactSet(bool staticCase, std::string const & contact_set_file_name, int numFrictionSides):
  ProblemDescriptor("ContactSet_2"),
  staticCase_(staticCase),
  m_numberOfFrictionSides(numFrictionSides),
  m_mass(1)
{
  if (staticCase)
    {
      m_dim = 2;
    }
  else
    {
      m_dim = 3;
    }
  loadContactSet(contact_set_file_name);

  int dot = contact_set_file_name.rfind(".");
  set_name(contact_set_file_name.substr(0, dot));
    
  // std::cout << "The second constructor for contactSet has been called!" << std::endl;
}

// ContactSet::ContactSet(const ContactSet& other):
//   ProblemDescriptor("ContactSet_copy"),
//   staticCase_(other.staticCase_),
//   m_numberOfFrictionSides(other.m_numberOfFrictionSides),
//   m_mass(other.m_mass),
//   m_dim(other.m_dim)
// {
//   for (auto contact: other.m_contacts)
//     {
//       m_contacts.push_back(contact);
//     }

//   for (auto acc: other.m_accelerations)
//     {
//       m_accelerations.push_back(acc);
//     }
// }

ContactSet::~ContactSet()
{
  // std::cout << "ContactSet destructor called!" << '\n';
}

void ContactSet::update()
{
  //std::cout << "The update method for contactSet has been called!" << std::endl;
  // std::cout << "Are the matrice sizes ok? " << (checkMatricesSizes() ? "yes" : "no") << std::endl;
  // if (checkMatricesSizes())
  //   {
      
  //   }
  updateMatricesSizes();
  
  setZeroMatrices();

  buildMatrixA();
  buildVectorB();
  buildFrictionF();
  buildFrictionVectorf();
}

bool ContactSet::checkMatricesSizes() const
{
  return (m_subCols == 3*m_contacts.size() and
	  m_globCols == m_subCols * m_accelerations.size() + m_dim and
	  m_subRows == (m_numberOfFrictionSides + 2)* m_contacts.size());
	  
}

void ContactSet::updateMatricesSizes()
{
  // Computing the new sizes
  m_subCols = 3*m_contacts.size();
  m_globCols = m_subCols * m_accelerations.size() + m_dim;

  m_subRows = (m_numberOfFrictionSides + 2)* m_contacts.size();
  m_globRows = m_subRows * m_accelerations.size() + 2 * m_dim;

  // resizing the matrices
  m_A.resize(6*m_accelerations.size(), m_globCols);
  m_b.resize(6*m_accelerations.size(), 1);

  m_F.resize(m_globRows, m_globCols);
  m_f.resize(m_globRows, 1);
}

void ContactSet::setZeroMatrices()
{
  m_A.setZero();
  m_b.setZero();
  m_F.setZero();
  m_f.setZero();  
}


void ContactSet::computeMatrixA1(Eigen::MatrixXd & A1)
{
  for(int i(0); i < m_contacts.size(); ++i)
  {
    A1.block<3, 3>(0, 3 * i) = Eigen::Matrix3d::Identity();
    A1.block<3, 3>(3, 3 * i) = skewSymmetric(m_contacts[i].get_position());
  }  
}

void ContactSet::computeMatrixA2(Eigen::MatrixXd & A2, Eigen::Vector3d const & acceleration)
{
  A2.block(0, 0, 3, m_dim) = Eigen::MatrixXd::Zero(3, m_dim);
  A2.block(3, 0, 3, m_dim) = -skewSymmetric(m_mass * acceleration).leftCols(m_dim);
  //A2.block(3, 0, 3, m_dim) = -skewSymmetric(acceleration).leftCols(m_dim);
}

void ContactSet::buildMatrixA()
{
  Eigen::MatrixXd tempA1;

  tempA1.setZero(6, m_subCols);
  computeMatrixA1(tempA1);

  for(int i = 0; i < m_accelerations.size(); ++i)
  {
    m_A.block(6 * i, m_subCols * i, 6, m_subCols) = tempA1;
    
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6, m_dim);
    computeMatrixA2(A2, m_accelerations[i]);
    m_A.block(6 * i, m_globCols - m_dim, 6, m_dim) = A2;

  }
}

void ContactSet::computeVectort(Eigen::VectorXd & t, Eigen::Vector3d const & acceleration)
{
  t.head(3) = -m_mass * acceleration;
  //t.head(3) = -acceleration;
  t.tail(3) = Eigen::Vector3d::Zero();
}

void ContactSet::buildVectorB()
{
  for(int i = 0; i < m_accelerations.size(); ++i)
  {
    Eigen::VectorXd t(6);
    computeVectort(t, m_accelerations[i]);
    m_b.segment<6>(6 * i) = t;
  }
}

void ContactSet::buildFrictionF()
{
  //std::cout << "The friction cones have " << m_numberOfFrictionSides << " sides" << std::endl;
  Eigen::MatrixXd F_contact(m_numberOfFrictionSides+2, 3);

  for(int i = 0; i < m_contacts.size(); ++i)
  {
    F_contact = m_contacts[i].linearizedFrictionCone(m_numberOfFrictionSides);

    //std::cout << "For contact " << i << "the linearized Friction Cone is: \n" << F_contact << std::endl;
      
    for(int j = 0; j < m_accelerations.size(); ++j)
    {
      m_F.block(j * (m_numberOfFrictionSides + 2) * m_contacts.size() + i * (m_numberOfFrictionSides + 2),
                j * 3 * m_contacts.size() + i * 3, (m_numberOfFrictionSides + 2), 3) = F_contact;
    }
  }

  m_F.block(m_globRows - 2*m_dim, m_globCols - m_dim, m_dim, m_dim) = Eigen::MatrixXd::Identity(m_dim, m_dim);
  m_F.bottomRightCorner(m_dim, m_dim) = -Eigen::MatrixXd::Identity(m_dim, m_dim);

 }

void ContactSet::buildFrictionVectorf()
{
  int ind = 0;
  for(int i = 0; i < m_contacts.size(); ++i)
  {
    for (int j = 0; j < m_accelerations.size(); ++j)
      {
	ind = j * m_contacts.size() * (m_numberOfFrictionSides + 2) + i * (m_numberOfFrictionSides + 2);
	m_f[ind] = m_contacts[i].fmax();
	m_f[ind + 1] = -m_contacts[i].fmin();
      }    
  }
  // Limitation of the CoM position
  for (int i = 2*m_dim; i>0; i--)
    {
      m_f[m_globRows-i] = 10;
    }

  if (m_dim == 3)
    {
      m_f[m_globRows-4]=2; // c_z max
      m_f[m_globRows-1]=0; //-c_z min
    }
}

Eigen::VectorXd ContactSet::Ydes() const
{
  // m_globCols = m_subCols * m_accelerations.size() + m_dim;
  // m_subCols = 3*m_contacts.size();

  Eigen::VectorXd subYdes = Eigen::VectorXd::Zero(m_subCols);
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  
  for (int i = 0; i<m_contacts.size(); i++)
    {
      if (m_contacts[i].isConstrained())
	{
	  f[2] = m_contacts[i].fmax();
	  rot = m_contacts[i].get_rotation();
	  subYdes.segment<3>(3*i) = rot * f;
	}
    }
  
  Eigen::VectorXd Ydes(m_globCols);
  for (int i = 0; i<m_accelerations.size(); i++)
    {
      Ydes.segment(m_subCols * i, m_subCols) = subYdes;
    }

  return Ydes;
}

Eigen::MatrixXd ContactSet::forcePos() const
{
  //   m_subCols = 3*m_contacts.size();
  // m_globCols = m_subCols * m_accelerations.size() + m_dim;

  // m_subRows = (m_numberOfFrictionSides + 2)* m_contacts.size();
  // m_globRows = m_subRows * m_accelerations.size() + 2 * m_dim;
  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(m_contacts.size() * m_accelerations.size(), m_globCols);
  Eigen::MatrixXd subF = Eigen::MatrixXd::Zero(m_contacts.size(), m_subCols);
  Eigen::Vector3d fMin;
  Eigen::Matrix3d rot;
  fMin << 0, 0, -1;
  
  for (int i = 0; i < m_contacts.size(); i++)
    {
      rot = m_contacts[i].get_rotation();
      subF.block<1, 3>(i, 3*i) = (rot * fMin).transpose();
    }

  for (int i = 0; i < m_accelerations.size(); i++)
    {
      F.block(i * m_contacts.size(), i * m_subCols, m_contacts.size(), m_subCols) = subF;
    }

  return F;
}

// ----------- input functions ----------
void ContactSet::loadContactSet(std::string const & file_name)
{
  tinyxml2::XMLDocument doc;
  doc.LoadFile(file_name.c_str());

  if(doc.ErrorID() == 0)
  {

    tinyxml2::XMLElement * root = doc.FirstChildElement("data");
    tinyxml2::XMLElement * mainXML = root->FirstChildElement("robot");
    tinyxml2::XMLElement * childXML(0);
    std::string mainType;
    std::string childType;

    Eigen::Vector3d acceleration;
    tinyxml2::XMLElement * lineXML(0);

    while(mainXML)
    {
      mainType = mainXML->Value();
      if(std::strcmp(mainType.c_str(), "robot")
         == 0) // FIX ME: for now in the XML files the contact set is still named robot
      {
        m_name = mainXML->Attribute("name");
        childXML = mainXML->FirstChildElement();

        while(childXML)
        {
          childType = childXML->Value();
          if(std::strcmp(childType.c_str(), "Mass") == 0)
          {
            childXML->QueryAttribute("mass", &m_mass);
          }
          else if(std::strcmp(childType.c_str(), "NumFeet") == 0)
          {
            // childXML->QueryAttribute("n_feet", &m_contacts.size());
          }
          else if(std::strcmp(childType.c_str(), "ContactPoint") == 0)
          {
            m_contacts.push_back(ContactPoints(childXML));
          }
          else
          {
            std::cerr << "Un-Recognized child element name: " << childType << '\n';
          }
          childXML = childXML->NextSiblingElement();
        }
      }
      else if(std::strcmp(mainType.c_str(), "accelerations") == 0)
      {
        childXML = mainXML->FirstChildElement();

        while(childXML)
        {
          // std::cout << "Add Acceleration!" << '\n';
          childType = childXML->Value();
          if(std::strcmp(childType.c_str(), "matrix") == 0)
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

  }
  else
  {
    std::cerr << "Failed to open the XML description file: " << file_name << std::endl;
  }
}

// ----------- output and display functions ----------
void ContactSet::showContactSet()
{
  std::cout << "Contact Set name: " << m_name << std::endl;
  std::cout << "Mass of the robot: " << m_mass << std::endl;
  std::cout << "Number of feet of the robot: " << m_contacts.size() << std::endl;

  for(auto contact : m_contacts)
  {
    std::cout << "Contact named: " << contact.get_name() << " (fmax=" << contact.fmax() << "N, fmin=" << contact.fmin()
              << "N)" << std::endl;
    //contact.showContactPoint();
  }

  std::cout << "Number of accelerations: " << get_numberOfAccelerations() << std::endl;
  for (auto acc: m_accelerations)
    {
      std::cout << acc.transpose() << std::endl;
    }
}

void ContactSet::saveContactSet(const std::string & file_name)
{
  // creating new xml object
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLDeclaration * declaration = doc.NewDeclaration();
  doc.InsertEndChild(declaration);

  // creating root node
  tinyxml2::XMLNode * root = doc.NewElement("data");
  doc.InsertEndChild(root);

  // building the robot element
  tinyxml2::XMLElement * XMLContactSet = doc.NewElement("robot");
  XMLContactSet->SetAttribute("name", m_name.c_str());

  {
    tinyxml2::XMLElement * XMLMass = doc.NewElement("Mass");
    XMLMass->SetAttribute("mass", m_mass);
    XMLContactSet->InsertEndChild(XMLMass);

    tinyxml2::XMLElement * XMLNumContacts = doc.NewElement("NumFeet");
    XMLNumContacts->SetAttribute("n_feet", (int) m_contacts.size());
    XMLContactSet->InsertEndChild(XMLNumContacts);

    for(auto foot : m_contacts)
    {
      XMLContactSet->InsertEndChild(foot.get_XMLContactPoint(doc));
    }
  }
  root->InsertEndChild(XMLContactSet);

  // building the accelerations element
  tinyxml2::XMLElement * XMLAccelerations = doc.NewElement("accelerations");
  std::string acc_name;

  for(int i = 0; i < m_accelerations.size(); i++)
  {
    tinyxml2::XMLElement * XMLAcc = doc.NewElement("matrix");
    acc_name = "acceleration_" + std::to_string(i);
    XMLAcc->SetAttribute("name", acc_name.c_str());
    XMLAcc->SetAttribute("row", 3);
    XMLAcc->SetAttribute("column", 1);
    {
      for(int j = 0; j < 3; j++)
      {
        tinyxml2::XMLElement * XMLline = doc.NewElement("line");
        tinyxml2::XMLElement * XMLv = doc.NewElement("v");
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

int ContactSet::get_contactIndexFromName(std::string contactName) const
{
  // TODO : use lambda function and algorithm
  int i(0), index(0);
  for(auto contact : m_contacts)
  {
    if(contact.get_name() == contactName)
    {
      index = i;
    }
    i++;
  }
  return index;
}

std::vector<std::string> ContactSet::get_contactNames() const
{
  // TODO : use lambda function and algorithm
  std::vector<std::string> names;
  for(auto contact : m_contacts)
  {
    names.push_back(contact.get_name());
  }
  return names;
}

bool ContactSet::hasContactNamed(std::string contactName) const
{
  // TODO : use lambda function and algorithm
  std::vector<std::string> names = get_contactNames();
  return find(names.begin(), names.end(), contactName) != names.end();
}

double ContactSet::contactFMax(std::string contactName) const
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);
      
      return m_contacts[ind].fmax();
    }
  else
    {
      return 0;
    }
}

double ContactSet::contactFMin(std::string contactName) const
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);
      
      return m_contacts[ind].fmin();
    }
  else
    {
      return 0;
    }
}

Eigen::Matrix4d ContactSet::contactHomTrans(std::string contactName) const
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);

      return m_contacts[ind].get_homTrans();
    }
  else
    {
      throw(42);// instead of returning 0 throwing is better
    }
}

bool ContactSet::hasConstrainedContact() const
{
  auto pred =  [](ContactPoints cp){
    return cp.isConstrained();
  };

  return std::find_if(m_contacts.begin(), m_contacts.end(), pred)!=m_contacts.end();
}


std::vector<std::string> ContactSet::constrainedContactNames() const
{
  std::vector<std::string> names;
  for(auto contact : m_contacts)
  {
    if (contact.isConstrained())
      {
	names.push_back(contact.get_name());
      }
  }
  return names;
}

int ContactSet::numberConstrainedContacts() const
{
  return std::count_if(m_contacts.begin(), m_contacts.end(), [](auto c){
      return c.isConstrained();
    });
}

// ------------------ setter -----------------------
void ContactSet::translateContact(int contactIndex, Eigen::Vector3d translation)
{
  if(contactIndex >= 0 && contactIndex < m_contacts.size())
    {
      m_contacts[contactIndex].translate(translation);
    }
  else
    {
      std::cerr << "Error: the contact index is not valid" << '\n';
    }
}

void ContactSet::updateContact(int contactIndex, Eigen::Matrix4d homTrans)
{
  if(contactIndex >= 0 && contactIndex < m_contacts.size())
    {
      m_contacts[contactIndex].set_contact(homTrans);
    }
  else
    {
      std::cerr << "Error: the contact index is not valid" << '\n';
    }
}

void ContactSet::updateContact(std::string contactName, Eigen::Matrix4d homTrans)
{
  if(hasContactNamed(contactName))
  {
    int index = get_contactIndexFromName(contactName);
    m_contacts[index].set_contact(homTrans);
  }
  else
  {
    std::cout << "There is no contact named " << contactName << "..." << std::endl;
    std::cout << "But I won't throw an error! hehehe!" << std::endl;
  }
}

void ContactSet::removeContact(int contactIndex)
{
  m_contacts.erase(m_contacts.begin() + contactIndex);
  //m_numberOfFeet--;
}

void ContactSet::removeContact(std::string contactName)
{
  int index = get_contactIndexFromName(contactName);
  removeContact(index);
}

void ContactSet::addContact(std::string contactName)
{
  m_contacts.push_back(ContactPoints(contactName, 0.5));
}

void ContactSet::addContact(std::string contactName, Eigen::Matrix4d homTrans, double friction, double fmax, double fmin, ContactType type)
{
  ContactPoints contact(contactName, friction, fmax, fmin, type);
  contact.set_contact(homTrans);
  m_contacts.push_back(contact);
}

void ContactSet::setContactFMax(double fmax, std::string contactName)
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);
      
      m_contacts[ind].fmax(fmax);
    }
  else
    {
      std::cerr << "Error: No such contact in the contactSet" << std::endl;
    }
}
void ContactSet::setContactFMin(double fmin, std::string contactName)
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);
      
      m_contacts[ind].fmin(fmin);
    }
  else
    {
      std::cerr << "Error: No such contact in the contactSet" << std::endl;
    }
}

void ContactSet::mass(double mass)
{
  const double eps = 0.00001;
  if (mass >= eps)
    {
      m_mass = mass;
    }
  else
    {
      std::cout << "The given mass is too small, the mass is set to the minimum: " << eps << std::endl;
      m_mass = eps;
    }
}

void ContactSet::updateContactType(std::string contactName, ContactType type)
{
  if (hasContactNamed(contactName))
    {
      int ind = get_contactIndexFromName(contactName);
      
      m_contacts[ind].contactType(type);
    }
  else
    {
      std::cout << "No contact named " << contactName << std::endl;
    }
}

void ContactSet::addCoMAcc(Eigen::Vector3d acceleration)
{
  m_accelerations.push_back(acceleration);
}

void ContactSet::printAcc()
{
  std::cout << "Accelerations: " << std::endl;
  for (auto acc: m_accelerations)
    {
      std::cout << acc.transpose() << std::endl;
    }
  std::cout << std::endl;
}


// ---------- Static function -----------

Eigen::Matrix3d ContactSet::skewSymmetric(Eigen::Vector3d const & vect)
{
  Eigen::Matrix3d vect_hat;
  vect_hat << 0, -vect(2), vect(1), vect(2), 0, -vect(0), -vect(1), vect(0), 0;
  return vect_hat;
}


