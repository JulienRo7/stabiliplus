#include "face.h"

#include "edge.h" // Because of forward declaration

int Face::GlobalFaceCounter = 0;
Face::Face(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Edge* edge1, Edge* edge2, Edge* edge3, Eigen::Vector3d innerPoint):
    m_index(GlobalFaceCounter),
    m_vertex1(vertex1), m_vertex2(vertex2), m_vertex3(vertex3),
    m_edge1(edge1), m_edge2(edge2), m_edge3(edge3)
{
    ++GlobalFaceCounter;

    m_edge1->addFace(this);
    m_edge2->addFace(this);
    m_edge3->addFace(this);

    Eigen::Vector3d p, q;
    p = m_vertex3->get_coordinates()-m_vertex1->get_coordinates();
    q = m_vertex2->get_coordinates()-m_vertex1->get_coordinates();

    m_normal = p.cross(q).normalized();
    m_offset = m_normal.dot(m_vertex1->get_coordinates());

    if (!pointInHalfSapce(innerPoint))
    {
        m_normal *= -1;
        m_offset *= -1;
    }



}

Face::~Face()
{
    std::cout << "face: " << this << '\n';
    std::cout << "edge1: " << m_edge1 << '\n';
    m_edge1->removeFace(this);
    std::cout << "edge2: " << m_edge2 << '\n';
    m_edge2->removeFace(this);
    std::cout << "edge3: " << m_edge3 << '\n';
    m_edge3->removeFace(this);
    std::cout << "Done!" << '\n';
}

bool Face::isSame(Face const& b) const
{
    return m_index == b.m_index;
}

bool Face::pointInHalfSapce(Eigen::Vector3d const& point, double const eps) const
{
    return m_normal.dot(point)-m_offset <= eps;
}

// ----------- operators overloading ----------
bool operator==(Face const& a, Face const& b)
{
    return a.isSame(b);
}
