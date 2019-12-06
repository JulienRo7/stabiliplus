#include "polytope/outervertex.h"

#include "polytope/outeredge.h" // forward declaration

int OuterVertex::GlobalOuterVertexCounter = 0;

// ---------- constructors ----------
OuterVertex::OuterVertex(std::shared_ptr<OuterFace> face1,
                         std::shared_ptr<OuterFace> face2,
                         std::shared_ptr<OuterFace> face3)
: m_index(GlobalOuterVertexCounter)
{
  ++GlobalOuterVertexCounter;

  m_outerFaces.push_back(face1);
  m_outerFaces.push_back(face2);
  m_outerFaces.push_back(face3);

  computeCoordinates();
}

OuterVertex::OuterVertex(Eigen::Vector3d coordinates) : m_index(GlobalOuterVertexCounter), m_coordinates(coordinates)
{
  ++GlobalOuterVertexCounter;
}

// ---------- destructor ----------
OuterVertex::~OuterVertex() {}

// ---------- class methods ----------
void OuterVertex::computeCoordinates()
{
  Eigen::Matrix3d A;

  A.row(0) = m_outerFaces[0]->get_normal().transpose();
  A.row(1) = m_outerFaces[1]->get_normal().transpose();
  A.row(2) = m_outerFaces[2]->get_normal().transpose();

  // std::cout << "Matrix A:" << '\n' << A << '\n';s

  m_coordinates[0] = m_outerFaces[0]->get_offset();
  m_coordinates[1] = m_outerFaces[1]->get_offset();
  m_coordinates[2] = m_outerFaces[2]->get_offset();

  m_coordinates = A.colPivHouseholderQr().solve(
      m_coordinates); // maybe there is a more efficient way but there seems to be issues with A's properties for ldlt
}

bool OuterVertex::strictlyContainedInHalfspace(std::shared_ptr<OuterFace> outerFace, double const eps) const
{
  return outerFace->get_normal().dot(m_coordinates) - outerFace->get_offset() < eps;
}

std::vector<std::shared_ptr<OuterVertex>> OuterVertex::findNeighbors()
{
  std::vector<std::shared_ptr<OuterVertex>> neighbors;
  for(auto it : m_outerEdges)
  {
    neighbors.push_back(it->get_otherOuterVertex(shared_from_this()));
  }
  return neighbors;
}

// ---------- getters ----------
int OuterVertex::get_index() const
{
  return m_index;
}

Eigen::Vector3d OuterVertex::get_coordinates() const
{
  return m_coordinates;
}

std::vector<std::shared_ptr<OuterFace>> OuterVertex::get_outerFaces() const
{
  // std::cout << "Outer Vertex " << m_index << ": ";
  // for (auto it : m_outerFaces)
  // {
  //     std::cout << "Face " << it->get_index() << ", ";
  // }
  // std::cout << '\n';

  return m_outerFaces;
}

std::vector<std::shared_ptr<OuterEdge>> OuterVertex::get_outerEdges() const
{
  return m_outerEdges;
}

// ---------- setters ----------
void OuterVertex::add_outerEdge(std::shared_ptr<OuterEdge> outerEdge)
{
  if(find(m_outerEdges.begin(), m_outerEdges.end(), outerEdge) == m_outerEdges.end())
  {
    m_outerEdges.push_back(outerEdge);
  }
  else
  {
    std::cout << "Warning: the edge has already been added!" << '\n';
  }
}

void OuterVertex::remove_outerEdge(std::shared_ptr<OuterEdge> outerEdge)
{
  auto pos_point = find(m_outerEdges.begin(), m_outerEdges.end(), outerEdge);
  if(pos_point != m_outerEdges.end())
  {
    m_outerEdges.erase(pos_point);
  }
  else
  {
    std::cout << "Warning: the edge has already been removed!" << '\n';
  }
}

void OuterVertex::add_outerFace(std::shared_ptr<OuterFace> outerFace)
{
  if(find(m_outerFaces.begin(), m_outerFaces.end(), outerFace) == m_outerFaces.end())
  {
    m_outerFaces.push_back(outerFace);
  }
}
