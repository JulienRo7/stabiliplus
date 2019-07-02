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

    m_normal = p.cross(q);
    m_area = m_normal.norm();
    m_normal.normalize();

    m_offset = m_normal.dot(m_vertex1->get_coordinates());

    if (!pointInHalfSpace(innerPoint))
    {
        m_normal *= -1;
        m_offset *= -1;
    }



}

Face::~Face()
{
    // std::cout << "Destructor for face: " << m_index << '\n';
    // std::cout << "edge1: " << m_edge1 << '\n';
    m_edge1->removeFace(this);
    m_edge1 = 0;
    // std::cout << "edge2: " << m_edge2 << '\n';
    m_edge2->removeFace(this);
    m_edge2 = 0;
    // std::cout << "edge3: " << m_edge3 << '\n';
    m_edge3->removeFace(this);
    m_edge3 = 0;
    // std::cout << "Done!" << '\n';
}

bool Face::isSame(Face const& b) const
{
    return m_index == b.m_index;
}

bool Face::pointInHalfSpace(Eigen::Vector3d const& point, double const eps) const
{
    return m_normal.dot(point)-m_offset <= eps;
}

std::list<Face*> Face::findNeighbors()
{
    std::list<Face*> neighbors;

    neighbors.push_back(m_edge1->get_otherFace(this));
    neighbors.push_back(m_edge2->get_otherFace(this));
    neighbors.push_back(m_edge3->get_otherFace(this));

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

Edge* Face::get_edge1() const
{
    return m_edge1;
}

Edge* Face::get_edge2() const
{
    return m_edge2;
}

Edge* Face::get_edge3() const
{
    return m_edge3;
}

std::list<Edge*> Face::get_edges() const
{
    std::list<Edge*> edgesList;

    edgesList.push_back(get_edge1());
    edgesList.push_back(get_edge2());
    edgesList.push_back(get_edge3());

    return edgesList;
}

// ----------- setters ----------
void Face::set_area_null()
{
    m_area = 0;
}


// ----------- static functions ----------
bool Face::compareFacesArea(Face* faceA, Face* faceB)
{
    return faceA->get_area() < faceB->get_area();
}

// ----------- operators overloading ----------

bool operator==(Face const& a, Face const& b)
{
    return a.isSame(b);
}
