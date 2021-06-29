#include "polytope/staticPoint.h"

StaticPoint::StaticPoint(Eigen::Vector2d dir, Eigen::Vector2d vertex, StaticMeasure mesType)
: m_innerVertex(vertex), m_searchDir(dir), m_measureType(mesType)
{
}

StaticPoint::~StaticPoint()
{
  m_prec = nullptr;
  m_next = nullptr;
}

// ----- class's main methods -----
void StaticPoint::updateNormal()
{
  Eigen::Vector2d v2 = m_next->innerVertex();
  if((v2 - m_innerVertex).norm() < 0.0001)
  {
    // std::cout << "New vertex too close! Taking normal of next." << std::endl;
    m_normal = m_next->normal();
  }
  else
  {
    m_normal << -m_innerVertex(1) + v2(1), m_innerVertex(0) - v2(0);
    m_normal /= m_normal.norm();
  }
}

void StaticPoint::updateOuterVertex()
{
  Eigen::Vector2d v2 = m_next->innerVertex();
  Eigen::Vector2d d2 = m_next->searchDir();

  double a(m_searchDir(0)), b(m_searchDir(1)), c(d2(0)), d(d2(1));
  double det = a * d - b * c;
  double off1(m_innerVertex.transpose() * m_searchDir), off2(v2.transpose() * d2);

  m_outerVertex << d * off1 - b * off2, a * off2 - c * off1;
  m_outerVertex /= det;
}

void StaticPoint::updateMeasure()
{
  switch(m_measureType)
  {
    case AREA:
      computeArea();
      break;

    case SUPPORT:
      computeSupport();
      break;
  }
}

void StaticPoint::update()
{
  updateNormal();
  updateOuterVertex();
  updateMeasure();
}

void StaticPoint::computeArea()
{
  Eigen::Vector2d v1(m_outerVertex - m_innerVertex), v2(m_next->innerVertex() - m_innerVertex);
  m_measure = v1(0) * v2(1) - v1(1) * v2(0);
}

void StaticPoint::computeSupport()
{
  m_measure = m_normal.transpose() * (m_outerVertex - m_innerVertex);
}

// ----- outputs -----
void StaticPoint::writeToStream(std::ofstream & file_stream) const
{
  file_stream << "iv;" // iv = inner vertice
              << m_innerVertex(0) << ";" << m_innerVertex(1) << ";" << std::endl;
  file_stream << "sd;" // sd = searchDirection
              << m_searchDir(0) << ";" << m_searchDir(1) << ";" << std::endl;
  file_stream << "ov;" // ov = outer vertice
              << m_outerVertex(0) << ";" << m_outerVertex(1) << ";" << std::endl;
  file_stream << "no;" // no = normal
              << m_normal(0) << ";" << m_normal(1) << ";" << std::endl;
}

tinyxml2::XMLElement * StaticPoint::xmlStaticPoint(tinyxml2::XMLDocument & doc) const
{
  auto staticPointXML = doc.NewElement("staticPoint");

  auto vecToXML = [&doc, &staticPointXML](Eigen::Vector2d vec, std::string name){
    auto vecXML = doc.NewElement(name.c_str());
    vecXML->SetAttribute("x", vec(0));
    vecXML->SetAttribute("y", vec(1));
    staticPointXML->InsertEndChild(vecXML);
  };
  
  // inner vertex
  vecToXML(m_innerVertex, "InnerVertex");
  // search direction
  vecToXML(m_searchDir, "SearchDirection");
  // outer vertex
  vecToXML(m_outerVertex, "OuterVertex");
  // normal
  vecToXML(m_normal, "Normal");

  return staticPointXML;
  
}

Eigen::Vector4d StaticPoint::plane() const
{
  double offset = m_innerVertex.transpose() * m_normal;
  Eigen::Vector4d plane;
  plane << m_normal, 0, offset;

  return plane;
}

// ----- getters -----
std::shared_ptr<StaticPoint> StaticPoint::next() const
{
  return m_next;
}

std::shared_ptr<StaticPoint> StaticPoint::prec() const
{
  return m_prec;
}

// ----- setters -----

void StaticPoint::next(std::shared_ptr<StaticPoint> next_pt)
{
  if(next_pt != m_next)
  {
    m_next = next_pt;
    update();
  }
  else
  {
    std::cout << "next point already set" << std::endl;
  }
}

void StaticPoint::prec(std::shared_ptr<StaticPoint> prec_pt)
{
  if(prec_pt != m_next)
  {
    m_prec = prec_pt;
  }
  else
  {
    std::cout << "prec point already set" << std::endl;
  }
}

// ----- operator overload -----
bool StaticPoint::operator()(const std::shared_ptr<StaticPoint> p1, const std::shared_ptr<StaticPoint> p2)
{
  return p1->measure() < p2->measure();
}

// ----- tests -----
const bool StaticPoint::pointInHalfSpace(Eigen::Vector2d const &point, double const eps) const
{
  return m_normal.dot(point-m_innerVertex) < eps;
}