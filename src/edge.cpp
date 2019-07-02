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

    // std::cout << "Vertex 1: " << m_vertex1->get_index() << '\n';
    // std::cout << "Vertex 2: " << m_vertex2->get_index() << '\n';
}

Edge::~Edge()
{
    m_vertex1->removeInnerNeighbor(m_vertex2);
    m_vertex1 = 0;
    m_vertex2->removeInnerNeighbor(m_vertex1);
    m_vertex2 = 0;

    m_face1 = 0;
    m_face2 = 0;
}

bool Edge::isSame(const Edge& b) const
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
        std::cerr << "Error: The edge " << m_index << " already has two faces" << '\n';
        std::cerr << "Face " << m_face1->get_index() << " and " << m_face2->get_index() << " cannot add face " << face->get_index() << " ."<<'\n';
    }

}

void Edge::removeFace(Face* face)
{

    if (face == m_face1)
    {
        m_face1 = 0;
    }
    else if (face == m_face2)
    {
        m_face2 = 0;
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

Face* Edge::get_otherFace(Face* face) const
{
    if (face==m_face1)
    {
        return m_face2;
    }
    else
    {
        return m_face1;
    }
}

Vertex* Edge::get_vertex1() const
{
    return m_vertex1;
}

Vertex* Edge::get_vertex2() const
{
    return m_vertex2;
}

Face* Edge::get_face1() const
{
    return m_face1;
}
Face* Edge::get_face2() const
{
    return m_face2;
}

// ----------- operators overloading ----------
bool operator==(Edge const& a, Edge const& b)
{
    return a.isSame(b);
}
