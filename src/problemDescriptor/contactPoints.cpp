#include "problemDescriptor/contactPoints.h"

// using namespace std;

// ------- constructors and destructor -------
ContactPoints::ContactPoints()
: m_name(""), m_frictionCoef(1), m_rotation(Eigen::Matrix3d::Zero()), m_position(0, 0, 0), fmax_(1000), fmin_(0)
{
}

ContactPoints::ContactPoints(std::string name, double frictionCoef, double fmax, double fmin, ContactType type)
: m_name(name), m_frictionCoef(frictionCoef), m_rotation(Eigen::Matrix3d::Zero()), m_position(Eigen::Vector3d::Zero()),
  fmax_(fmax), fmin_(fmin), contactType_(type)
{
}

ContactPoints::ContactPoints(tinyxml2::XMLElement * contactPointXML) : fmax_(1000), fmin_(0), contactType_(support)
{

  tinyxml2::XMLElement * currentXMLElement = contactPointXML->FirstChildElement();
  tinyxml2::XMLElement * lineXML(0);
  tinyxml2::XMLElement * valueXML(0);
  std::string currentType;

  m_name = contactPointXML->Attribute("name");

  while(currentXMLElement)
  {
    currentType = currentXMLElement->Name();
    if(std::strcmp(currentType.c_str(), "friction") == 0)
    {
      // Get the friction coefficient of the contact Point
      currentXMLElement->QueryDoubleAttribute("mu", &m_frictionCoef);
    }
    else if(std::strcmp(currentType.c_str(), "flim") == 0)
    {
      // Get the fmax and fmin
      currentXMLElement->QueryDoubleAttribute("fmax", &fmax_);
      currentXMLElement->QueryDoubleAttribute("fmin", &fmin_);
    }
    else if(std::strcmp(currentType.c_str(), "contactType") == 0)
    {
      // Get the fmax and fmin
      std::string type;
      type = currentXMLElement->Attribute("type");

      if (type == "constrained")
	{
	  contactType_ = constrained;
	}
      else
	{
	  contactType_ = support;
	}
    }
    else if(std::strcmp(currentType.c_str(), "matrix") == 0)
    {
      // if the element if a matrix then it is either the rotation matrix or the position vector
      lineXML = currentXMLElement->FirstChildElement("line");
      if(std::strcmp(currentXMLElement->Attribute("name"), "rotation") == 0)
      {
        // case of the rotation matrix
        for(int i = 0; i < 3; i++)
        {
          valueXML = lineXML->FirstChildElement("v");
          for(int j = 0; j < 3; j++)
          {
            valueXML->QueryDoubleText(&m_rotation(i, j));
            valueXML = valueXML->NextSiblingElement("v");
          }
          lineXML = lineXML->NextSiblingElement("line");
        }
      }
      else if(std::strcmp(currentXMLElement->Attribute("name"), "position") == 0)
      {
        // case of the position vector
        for(int i = 0; i < 3; i++)
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
    else if(std::strcmp(currentType.c_str(), "rotation") == 0)
    {
      // Get the friction coefficient of the contact Point
      double phi, theta, psi;
      currentXMLElement->QueryDoubleAttribute("phi", &phi);
      currentXMLElement->QueryDoubleAttribute("theta", &theta);
      currentXMLElement->QueryDoubleAttribute("psi", &psi);

      /* Proper Euler Angles X->Z->X */

      double const c1 = cos(phi);
      double const s1 = sin(phi);

      double const c2 = cos(theta);
      double const s2 = sin(theta);

      double const c3 = cos(psi);
      double const s3 = sin(psi);

      m_rotation(0, 0) = c2;
      m_rotation(0, 1) = -s2 * c3;
      m_rotation(0, 2) = s2 * s3;
      m_rotation(1, 0) = c1 * s2;
      m_rotation(1, 1) = c1 * c2 * c3 - s1 * s3;
      m_rotation(1, 2) = -c1 * c2 * s3 - s1 * c3;
      m_rotation(2, 0) = s1 * s2;
      m_rotation(2, 1) = s1 * c2 * c3 + c1 * s3;
      m_rotation(2, 2) = -s1 * c2 * s3 + c1 * c3;
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

ContactPoints::~ContactPoints() {}

// ------- class' main methods -------
void ContactPoints::showContactPoint()
{
  std::cout << "Contact Point Homogeneous Transform:" << '\n';
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      std::cout << m_rotation(i, j) << ' ';
    }
    std::cout << m_position(i) << '\n';
  }
  std::cout << "0 0 0 1" << '\n';
}

Eigen::MatrixXd ContactPoints::linearizedFrictionCone(int numberOfFrictionSides)
{
  Eigen::MatrixXd F(numberOfFrictionSides + 2, 3);
  Eigen::RowVector3d line;

  line << 0, 0, 1;
  line = line * (m_rotation.transpose());
  F.row(0) = line;

  line << 0, 0, -1;
  line = line * (m_rotation.transpose());
  F.row(1) = line;

  double Dtheta(2 * M_PI / numberOfFrictionSides);
  // std::cout << "Dtheta: " << Dtheta << '\n';
  for(int i = 0; i < numberOfFrictionSides; ++i)
  {
    line << cos(i * Dtheta), sin(i * Dtheta), -m_frictionCoef * cos(Dtheta / 2);
    // line << cos(i*Dtheta), sin(i*Dtheta), -m_frictionCoef;
    line = line * (m_rotation.transpose());
    F.row(i + 2) = line;
  }

  return F;
}

// ------- getters -------
std::string ContactPoints::get_name() const
{
  return m_name;
}

Eigen::Vector3d ContactPoints::get_position() const
{
  return m_position;
}

Eigen::Matrix3d ContactPoints::get_rotation() const
{
  return m_rotation;
}

Eigen::Matrix4d ContactPoints::get_homTrans() const
{
  Eigen::Matrix4d homTrans = Eigen::Matrix4d::Zero();
  homTrans.topLeftCorner<3, 3>() = m_rotation;
  homTrans.rightCols<1>().head<3>() = m_position;

  return homTrans;
}

tinyxml2::XMLElement * ContactPoints::get_XMLContactPoint(tinyxml2::XMLDocument & doc) const
{
  tinyxml2::XMLElement * XMLContactPoint = doc.NewElement("ContactPoint");
  XMLContactPoint->SetAttribute("name", m_name.c_str());

  {
    tinyxml2::XMLElement * XMLfriction = doc.NewElement("friction");
    XMLfriction->SetAttribute("mu", m_frictionCoef);
    XMLContactPoint->InsertEndChild(XMLfriction);

    tinyxml2::XMLElement * XMLforces = doc.NewElement("flim");
    XMLforces->SetAttribute("fmax", fmax_);
    XMLforces->SetAttribute("fmin", fmin_);
    XMLContactPoint->InsertEndChild(XMLforces);
    
    tinyxml2::XMLElement * XMLContactType = doc.NewElement("contactType");
    std::string type;
    switch (contactType_)
      {
      case support:
	type = "support";
	break;
      case constrained:
	type = "constrained";
	break;
      default:
	type = "support";
      }
    XMLContactType->SetAttribute("type", type.c_str());
    XMLContactPoint->InsertEndChild(XMLContactType);
    
    tinyxml2::XMLElement * XMLPosition = doc.NewElement("matrix");
    XMLPosition->SetAttribute("name", "position");
    XMLPosition->SetAttribute("row", 3);
    XMLPosition->SetAttribute("column", 1);
    {
      for(int i = 0; i < 3; i++)
      {
        tinyxml2::XMLElement * XMLline = doc.NewElement("line");
        tinyxml2::XMLElement * XMLv = doc.NewElement("v");
        XMLv->SetText(m_position[i]);
        XMLline->InsertEndChild(XMLv);
        XMLPosition->InsertEndChild(XMLline);
      }
    }
    XMLContactPoint->InsertEndChild(XMLPosition);

    tinyxml2::XMLElement * XMLRotation = doc.NewElement("matrix");
    XMLRotation->SetAttribute("name", "rotation");
    XMLRotation->SetAttribute("row", 3);
    XMLRotation->SetAttribute("column", 3);
    {
      for(int i = 0; i < 3; i++)
      {
        tinyxml2::XMLElement * XMLline = doc.NewElement("line");
        for(int j = 0; j < 3; j++)
        {
          tinyxml2::XMLElement * XMLv = doc.NewElement("v");
          XMLv->SetText(m_rotation(i, j));
          XMLline->InsertEndChild(XMLv);
        }
        XMLRotation->InsertEndChild(XMLline);
      }
    }
    XMLContactPoint->InsertEndChild(XMLRotation);
  }

  return XMLContactPoint;
}

double ContactPoints::fmax() const
{
  return fmax_;
}

double ContactPoints::fmin() const
{
  return fmin_;
}

// ------- setters -------
void ContactPoints::translate(Eigen::Vector3d trans)
{
  m_position += trans;
}

void ContactPoints::set_contact(Eigen::Matrix4d homTrans)
{
  m_rotation = homTrans.topLeftCorner<3, 3>();
  m_position = homTrans.rightCols<1>().head<3>();
}

void ContactPoints::set_friction(double frictionCoef)
{
  if(frictionCoef >= 0)
  {
    m_frictionCoef = frictionCoef;
  }
}

void ContactPoints::fmax(double f)
{
  if(f >= 0)
  {
    fmax_ = f;
  }
  else
  {
    fmax_ = 0;
    std::cerr << "Error: fmax should be positive" << std::endl;
  }
}

void ContactPoints::fmin(double f)
{
  if(f >= 0)
  {
    fmin_ = f;
  }
  else
  {
    fmin_ = 0;
    std::cerr << "Error: fmin should be positive" << std::endl;
  }
}

void ContactPoints::contactType(ContactType type)
{
  contactType_ = type;
}
