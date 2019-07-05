#ifndef OUTERVETEX_H_INCLUDED
#define OUTERVETEX_H_INCLUDED

#include <iostream>
#include <vector>

#include "outerface.h"

#include <Eigen/Dense>

class OuterEdge; // Forward declaration

class OuterVertex
{
public:

    // ---------- constructors ----------
    OuterVertex(OuterFace* face1, OuterFace* face2, OuterFace* face3);
    OuterVertex(Eigen::Vector3d coordinates);

    // ---------- destructor ----------
    ~OuterVertex();

    // ---------- class methods ----------
    void computeCoordinates();
    bool strictlyContainedInHalfspace(OuterFace* outerFace, double const eps=0.0) const;

    std::vector<OuterVertex*> findNeighbors();
    // ---------- getters ----------
    int get_index() const;
    Eigen::Vector3d get_coordinates() const;
    std::vector<OuterFace*> get_outerFaces() const;
    std::vector<OuterEdge*> get_outerEdges() const;

    // ---------- setters ----------
    void add_outerEdge(OuterEdge* outerEgde);
    void remove_outerEdge(OuterEdge* outerEdge);
    void add_outerFace(OuterFace* outerFace);

private:

    static int GlobalOuterVertexCounter;
    int m_index;

    Eigen::Vector3d m_coordinates;

    std::vector<OuterFace*> m_outerFaces;
    std::vector<OuterEdge*> m_outerEdges;

};


#endif // OUTERVETEX_H_INCLUDED
