#include "polytope/vertex.h"

int Vertex::GlobalVertexCounter = 0;

Vertex::Vertex(Eigen::Vector3d coordinates, Eigen::Vector3d direction)
: m_index(GlobalVertexCounter), m_coordinates(coordinates), m_direction(direction)
{
  ++GlobalVertexCounter;
  m_offset = m_direction.dot(m_coordinates);
}

Vertex::~Vertex()
{
  for(auto it : m_innerNeighbors)
  {
    it = nullptr; // set the pointers to 0 -> I don't think it is usefull but anyway
  }
}

void Vertex::addInnerNeighbor(std::shared_ptr<Vertex> const neigh)
{
  m_innerNeighbors.push_back(neigh);
}

void Vertex::removeInnerNeighbor(std::shared_ptr<Vertex> const neigh)
{
  std::remove(m_innerNeighbors.begin(), m_innerNeighbors.end(), neigh);
}

bool Vertex::isSame(Vertex const & b) const
{
  return m_index == b.m_index;
}

// ----------- getters ----------
int Vertex::get_index() const
{
  return m_index;
}

Eigen::Vector3d Vertex::get_coordinates() const
{
  return m_coordinates;
}

Eigen::Vector3d Vertex::get_direction() const
{
  return m_direction;
}

double Vertex::get_offset() const
{
  return m_offset;
}

// ----------- getters ----------
void Vertex::set_coordinates(Eigen::Vector3d coord)
{
  m_coordinates = coord;
}

// ----------- operators overloading ----------
bool operator==(Vertex const & a, Vertex const & b)
{
  return a.isSame(b);
}
