#ifndef OUTEREDGE_H_INCLUDED
#define OUTEREDGE_H_INCLUDED

#include <iostream>
#include <vector>

#include "outervertex.h"
#include "outerface.h"

#include <Eigen/Dense>

// class Edge; // Forward declaration

class OuterEdge
{
public:

    // ---------- constructors ----------
    OuterEdge(OuterVertex* outerVertex1, OuterVertex* outerVertex2);

    // ---------- destructor ----------
    ~OuterEdge();

    // ---------- getters ----------
    int get_index() const;
    OuterVertex* get_outerVertex1() const;
    OuterVertex* get_outerVertex2() const;
    OuterVertex* get_otherOuterVertex(OuterVertex* OuterVertex) const;

    OuterFace* get_outerFace1() const;
    OuterFace* get_outerFace2() const;
    void add_outerFacesToVector(std::vector<OuterFace*> *vectorOuterFaces);

    // ---------- setters ----------
    void set_outerVertice1(OuterVertex* outerVertex);
    void set_outerVertice2(OuterVertex* outerVertex);
    void switch_outerVertices(OuterVertex* oldVertex, OuterVertex* newVertex);

private:

    static int GlobalOuterEdgeCounter;
    int m_index;

    OuterVertex* m_outerVertex1;
    OuterVertex* m_outerVertex2;

    OuterFace* m_outerFace1;
    OuterFace* m_outerFace2;

};


#endif // OUTEREDGE_H_INCLUDED
