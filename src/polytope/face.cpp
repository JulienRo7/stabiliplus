#include "polytope/face.h"

#include "polytope/edge.h" // Because of forward declaration

int Face::GlobalFaceCounter = 0;

Face::Face(std::shared_ptr<Vertex> vertex1,
           std::shared_ptr<Vertex> vertex2,
           std::shared_ptr<Vertex> vertex3,
           std::shared_ptr<Edge> edge1,
           std::shared_ptr<Edge> edge2,
           std::shared_ptr<Edge> edge3,
           Eigen::Vector3d innerPoint)
: m_index(GlobalFaceCounter), m_vertex1(vertex1), m_vertex2(vertex2), m_vertex3(vertex3), m_edge1(edge1),
  m_edge2(edge2), m_edge3(edge3), m_supportFunction(-1)
{
  ++GlobalFaceCounter;

  Eigen::Vector3d p, q;
  p = m_vertex3->get_coordinates() - m_vertex1->get_coordinates();
  q = m_vertex2->get_coordinates() - m_vertex1->get_coordinates();

  m_normal = p.cross(q);
  m_area = m_normal.norm();
  m_normal.normalize();

  m_offset = m_normal.dot(m_vertex1->get_coordinates());

  if(!pointInHalfSpace(innerPoint))
  {
    m_normal *= -1;
    m_offset *= -1;
  }
}

Face::~Face()
{
  // std::cout << "Destructor called for face " << m_index << '\n';
  // std::cout << "Destructor for face: " << m_index << '\n';
  // std::cout << "edge1: " << m_edge1 << '\n';
  m_edge1 = nullptr;
  // std::cout << "edge2: " << m_edge2 << '\n';
  m_edge2 = nullptr;
  // std::cout << "edge3: " << m_edge3 << '\n';
  m_edge3 = nullptr;
  // std::cout << "Done!" << '\n';

  m_supportPoint = 0;
}

bool Face::init() // this function needs to be called after creating a new face because shared_from_this cannot be used
                  // in the constructor.
{
  if(!m_edge1->addFace(shared_from_this())){return false;};
  if(!m_edge2->addFace(shared_from_this())){return false;};
  if(!m_edge3->addFace(shared_from_this())){return false;};
  return true;
}

void Face::finish() // this function needs to be called before destroying a face because shared_from_this cannot be used
                    // in the destructor.
{
  m_edge1->removeFace(shared_from_this());
  m_edge2->removeFace(shared_from_this());
  m_edge3->removeFace(shared_from_this());
}

bool Face::isSame(Face const & b) const
{
  return m_index == b.m_index;
}

bool Face::pointInHalfSpace(Eigen::Vector3d const & point, double const eps) const
{
  return m_normal.dot(point) - m_offset <= eps;
}

std::vector<std::shared_ptr<Face>> Face::findNeighbors()
{
  std::vector<std::shared_ptr<Face>> neighbors;

  neighbors.push_back(m_edge1->get_otherFace(shared_from_this()));
  neighbors.push_back(m_edge2->get_otherFace(shared_from_this()));
  neighbors.push_back(m_edge3->get_otherFace(shared_from_this()));

  return neighbors;
}

// ----------- getters ----------
int Face::get_index() const
{
  return m_index;
}

double Face::get_area() const
{
  return m_area;
}

double Face::get_offset() const
{
  return m_offset;
}

Eigen::Vector3d Face::get_normal() const
{
  return m_normal;
}

std::shared_ptr<Edge> Face::get_edge1() const
{
  return m_edge1;
}

std::shared_ptr<Edge> Face::get_edge2() const
{
  return m_edge2;
}

std::shared_ptr<Edge> Face::get_edge3() const
{
  return m_edge3;
}

std::vector<std::shared_ptr<Edge>> Face::get_edges() const
{
  std::vector<std::shared_ptr<Edge>> edgesvector;

  edgesvector.push_back(get_edge1());
  edgesvector.push_back(get_edge2());
  edgesvector.push_back(get_edge3());

  return edgesvector;
}

std::shared_ptr<Vertex> Face::get_vertex1() const
{
  return m_vertex1;
}
std::shared_ptr<Vertex> Face::get_vertex2() const
{
  return m_vertex2;
}
std::shared_ptr<Vertex> Face::get_vertex3() const
{
  return m_vertex3;
}

std::shared_ptr<OuterVertex> Face::get_supportPoint() const
{
  return m_supportPoint;
}

double Face::get_supportFunction() const
{
  return m_supportFunction;
}

double Face::get_measure() const
{
  if(m_area == 0)
  {
    return 0;
  }
  else
  {
    return m_supportFunction;
  }
}

// ----------- setters ----------
void Face::set_area_null()
{
  m_area = 0;
}

void Face::set_supportPoint(std::shared_ptr<OuterVertex> supportPoint)
{
  m_supportPoint = supportPoint;
  m_supportFunction = m_normal.dot(supportPoint->get_coordinates()) - m_offset;
}

// ----------- static functions ----------
bool Face::compareFacesMeasure(std::shared_ptr<Face> faceA, std::shared_ptr<Face> faceB)
{
  return faceA->get_measure() < faceB->get_measure();
}

// ----------- operators overloading ----------

bool operator==(Face const & a, Face const & b)
{
  return a.isSame(b);
}
