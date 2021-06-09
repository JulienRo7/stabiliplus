#include "polytope/edge.h"

#include "polytope/face.h" // Because of forward declaration

int Edge::GlobalEdgeCounter = 0;

Edge::Edge(std::shared_ptr<Vertex> vertex1, std::shared_ptr<Vertex> vertex2)
: m_index(GlobalEdgeCounter), m_vertex1(vertex1), m_vertex2(vertex2)
{

  ++GlobalEdgeCounter;
  m_vertex1->addInnerNeighbor(m_vertex2);
  m_vertex2->addInnerNeighbor(m_vertex1);

  // m_face1 = 0;
  // m_face2 = 0;

  // std::cout << "Vertex 1: " << m_vertex1->get_index() << '\n';
  // std::cout << "Vertex 2: " << m_vertex2->get_index() << '\n';
}

Edge::~Edge()
{
  m_vertex1->removeInnerNeighbor(m_vertex2);
  // m_vertex1 = 0;
  m_vertex2->removeInnerNeighbor(m_vertex1);
  // m_vertex2 = 0;

  // m_face1 = 0;
  // m_face2 = 0;
}

bool Edge::isSame(const Edge & b) const
{
  return m_index == b.m_index;
}

bool Edge::addFace(const std::shared_ptr<Face> & face)
{

  if(!m_face1)
  {
    m_face1 = face;
    return true;
  }
  else if(!m_face2)
  {
    m_face2 = face;
    return true;
  }
  else
  {
    return false;
    std::cerr << "Error: The edge " << m_index << " already has two faces" << '\n';
    std::cerr << "Face " << m_face1->get_index() << " and " << m_face2->get_index() << " cannot add face "
              << face->get_index() << " ." << '\n';
  }
}

void Edge::removeFace(const std::shared_ptr<Face> & face)
{
  if(face == m_face1)
  {
    m_face1 = nullptr;
  }
  else if(face == m_face2)
  {
    m_face2 = nullptr;
  }
  // else
  // {
  //     std::cerr << "Error: the face is not on the side of the edge" << '\n';
  // }
}

// ----------- getters ----------

int Edge::get_index() const
{
  return m_index;
}

std::shared_ptr<Face> Edge::get_otherFace(std::shared_ptr<Face> face) const
{
  if(face == m_face1)
  {
    return m_face2;
  }
  else
  {
    return m_face1;
  }
}

std::shared_ptr<Vertex> Edge::get_vertex1() const
{
  return m_vertex1;
}

std::shared_ptr<Vertex> Edge::get_vertex2() const
{
  return m_vertex2;
}

std::shared_ptr<Face> Edge::get_face1() const
{
  return m_face1;
}
std::shared_ptr<Face> Edge::get_face2() const
{
  return m_face2;
}

// ----------- operators overloading ----------
bool operator==(Edge const & a, Edge const & b)
{
  return a.isSame(b);
}
