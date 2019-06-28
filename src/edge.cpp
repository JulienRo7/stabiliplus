#include "edge.h"

#include "face.h" // Because of forward declaration

int Edge::GlobalEdgeCounter = 0;

Edge::Edge(Vertex* vertex1, Vertex* vertex2): m_index(GlobalEdgeCounter),
    m_vertex1(vertex1), m_vertex2(vertex2)
{
    ++GlobalEdgeCounter;
    m_vertex1->addInnerNeighbor(m_vertex2);
    m_vertex2->addInnerNeighbor(m_vertex1);

    m_face1 = 0;
    m_face2 = 0;

}

Edge::~Edge()
{
    m_vertex1->removeInnerNeighbor(m_vertex2);
    m_vertex2->removeInnerNeighbor(m_vertex1);
}

bool Edge::isSame(Edge const& b) const
{
    return m_index == b.m_index;
}

void Edge::addFace(Face* face)
{
    if (m_face1 == 0)
    {
        m_face1 = face;
    }
    else if (m_face2 == 0)
    {
        m_face2 = face;
    }
    else
    {
        std::cerr << "Error: The edge already has two faces" << '\n';
    }

}

void Edge::removeFace(Face* face)
{
    if (*face == *m_face1)
    {
        m_face1 = 0;
    }
    else if (*face == *m_face2)
    {
        m_face2 = 0;
    }
    else
    {
        std::cerr << "Error: the face is not on the side of the edge" << '\n';
    }
}

// ----------- operators overloading ----------
bool operator==(Edge const& a, Edge const& b)
{
    return a.isSame(b);
}
