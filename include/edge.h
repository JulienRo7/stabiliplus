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

    bool isSame(Edge const& b) const;

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
