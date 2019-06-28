#include "vertex.h"

int Vertex::GlobalVertexCounter = 0;

Vertex::Vertex(Eigen::Vector3d coordinates, Eigen::Vector3d direction):
    m_index(GlobalVertexCounter),
    m_coordinates(coordinates),
    m_direction(direction)
{
    ++GlobalVertexCounter;
    m_offset = m_direction.dot(m_coordinates);
}

Vertex::~Vertex()
{

}

void Vertex::addInnerNeighbor(Vertex* const neigh)
{
    m_innerNeighbors.push_back(neigh);
}

void Vertex::removeInnerNeighbor(Vertex* const neigh)
{
    std::remove(m_innerNeighbors.begin(), m_innerNeighbors.end(), neigh);
}


bool Vertex::isSame(Vertex const& b) const
{
    return m_index == b.m_index;
}

// ----------- getters ----------
Eigen::Vector3d Vertex::get_coordinates() const
{
    return m_coordinates;
}

Eigen::Vector3d Vertex::get_direction() const
{
    return m_direction;
}


// ----------- operators overloading ----------
bool operator==(Vertex const& a, Vertex const& b)
{
    return a.isSame(b);
}
