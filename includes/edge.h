#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED

#include "vertex.h"

// #include "face.h"
class Face; // Forward declaration

class Edge
{
public:
    Edge(Vertex* vertex1, Vertex* vertex2);

    ~Edge();

    void addFace(Face* face);
    void removeFace(Face* Face);

    bool isSame(const Edge& b) const;

    // ----------- getters ----------
    int get_index() const;
    Face* get_otherFace(Face* Face) const;
    
    Vertex* get_vertex1() const;
    Vertex* get_vertex2() const;

    Face* get_face1() const;
    Face* get_face2() const;

private:
    static int GlobalEdgeCounter;
    int m_index;

    Vertex *m_vertex1;
    Vertex *m_vertex2;

    Face *m_face1;
    Face *m_face2;

};

bool operator==(Edge const& a, Edge const& b);

#endif // EDGE_H_INCLUDED
