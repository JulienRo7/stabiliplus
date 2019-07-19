#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef STABILITY_POLYTOPE_H_INCLUDE
#define STABILITY_POLYTOPE_H_INCLUDE

// standart libraries
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <cmath>
#include <algorithm>

#include <chrono>
#include <memory>

// libraries
#include <Eigen/Dense>
#include <glpk.h>

// custom libraries
#include "robot.h"

#include "vertex.h"
#include "edge.h"
#include "face.h"
#include "outervertex.h"
#include "outeredge.h"
#include "outerface.h"

class StabilityPolytope
{
public:
    // ----------- constructors and destructor ----------
    StabilityPolytope(Robot robot, int maxNumberOfIteration = 50);
    ~StabilityPolytope();

    // ----------- main class methods ----------
    void buildStabilityProblem(); // compute the reduced stability problem
    void solveStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point);

    void projectionStabilityPolyhedron();

    Eigen::Vector3d computeInnerPoint();
    void buildInnerPoly();
    void updateInnerPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);
    void buildOuterPoly();
    void updateOuterPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);

    void updateSupportFunctions(std::shared_ptr<Face>& dirFace);
    bool computeSupportFunction(std::shared_ptr<Face>& face, const std::shared_ptr<OuterVertex>& initPoint);

    double computeResidualFromScratch();
    bool stopCriterion();

    // ----------- output and display functions ----------
    void exportVertices();
    void showPoly();

    // ----------- getters ----------
    int get_numberOfVertices() const;
    int get_numberOfFaces() const;
    int get_numberOfOuterVertices() const;
    int get_numberOfOuterFaces() const;

    double get_lpMicro() const;
    double get_innerConvexMicro() const;
    double get_outerConvexMicro() const;
    double get_supportFunctionMicro() const;

    // ----------- setters ----------
    void set_maxNumberOfIterations(int maxNumberOfIteration);

    // ---------- static functions ---------

private:
    // robot:
    Robot m_robot; // the robot should not change

    // attributes used for the LP problem
    glp_prob *m_lp;
    Eigen::MatrixXd m_Q_u;
    Eigen::MatrixXd m_Q_c;
    Eigen::MatrixXd m_R_inv_T_b;

    int m_numberOfIterations;
    double m_residual;
    // inner polyhedron
    std::vector<std::shared_ptr<Vertex>> m_vertices;
    Eigen::Vector3d m_innerPoint;

    std::vector<std::shared_ptr<Edge>> m_edges;
    std::vector<std::shared_ptr<Face>> m_faces;

    // outer polyhedron
    std::vector<std::shared_ptr<OuterVertex>> m_outerVertices;
    std::vector<std::shared_ptr<OuterEdge>> m_outerEdges;
    std::vector<std::shared_ptr<OuterFace>> m_outerFaces;

    std::map<std::shared_ptr<Vertex>, std::shared_ptr<OuterFace>> m_innerOuterLink;

    // options
    int m_maxNumberOfIteration;

    // time measures
    double m_lpMicro;
    double m_innerConvexMicro;
    double m_outerConvexMicro;
    double m_supportFunctionMicro;


}; // class StabilityPolytope

#endif // STABILITY_POLYTOPE_H_INCLUDE
