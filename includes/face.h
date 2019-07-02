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
    bool pointInHalfSpace(Eigen::Vector3d const& point, double const eps = 0.0) const;

    std::list<Face*> findNeighbors();

    // ---------- getters ----------
    int get_index() const;
    double get_area() const;
    double get_offset() const;

    Eigen::Vector3d get_normal() const;
    Edge* get_edge1() const;
    Edge* get_edge2() const;
    Edge* get_edge3() const;
    std::list<Edge*> get_edges() const;

    // ---------- setters ----------
    void set_area_null();

    // ---------- static functions ----------
    static bool compareFacesArea(Face* faceA, Face* faceB);

private:

    static int GlobalFaceCounter;
    int m_index;

    Eigen::Vector3d m_normal;
    double m_offset;
    double m_area;

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
