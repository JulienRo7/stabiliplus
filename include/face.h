#ifndef FACE_H_INCLUDED
#define FACE_H_INCLUDED

#include <iostream>
#include <list>

#include "vertex.h"

#include <Eigen/Dense>

class Edge; // Forward declaration

class Face
{
public:
    Face(Vertex* vertex1, Vertex* vertex2, Vertex* vertex3, Edge* edge1, Edge* edge2, Edge* edge3, Eigen::Vector3d innerPoint);
    ~Face();

    bool isSame(Face const& b) const;
    /*
    Note: it is considered that the normal is toward the outside
    */
    bool pointInHalfSapce(Eigen::Vector3d const& point, double const eps = 0.0) const;
    std::list<Face*> findNeighbors();


private:

    static int GlobalFaceCounter;
    int m_index;

    Eigen::Vector3d m_normal;
    double m_offset;

    Vertex *m_vertex1;
    Vertex *m_vertex2;
    Vertex *m_vertex3;

    Edge *m_edge1;
    Edge *m_edge2;
    Edge *m_edge3;

};

// ----------- operators overloading ----------
bool operator==(Face const& a, Face const& b);

#endif // FACE_H_INCLUDED
