#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

/*
The robot class contains the description of the robot. It can load it and make other things
*/
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
#include <tinyxml2.h>
#include <Eigen/Dense>
#include <glpk.h>

// custom libraries
#include "contactPoints.h"
#include "vertex.h"
#include "edge.h"
#include "face.h"
#include "outervertex.h"
#include "outeredge.h"
#include "outerface.h"

class Robot
{

public:
    Robot();
    Robot(std::string const& robot_file_name, int numFrictionSides=8, int maxNumberOfIteration = 50);
    ~Robot();

    void loadRobot(std::string const& file_name);
    void showRobot();

    Eigen::MatrixXd computeMatrixA1();
    Eigen::MatrixXd computeMatrixA2(Eigen::Vector3d const& acceleration);
    Eigen::VectorXd computeVector_t(Eigen::Vector3d const& acceleration);

    Eigen::MatrixXd buildMatrixA();
    Eigen::VectorXd buildVectorB();
    Eigen::MatrixXd buildFrictionF();
    Eigen::VectorXd buildFrictionVectorf();


    void buildStabilityProblem();
    void buildReducedStabilityProblem();
    void solveStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point);
    void solveReducedStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point);

    void projectionStabilityPolyhedron();

    Eigen::Vector3d computeInnerPoint();
    void buildInnerPoly();
    void updateInnerPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);
    void buildOuterPoly();
    void updateOuterPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);

    void computeSupportFunctions(std::shared_ptr<Face>& dirFace);
    bool computeSupportFunction(std::shared_ptr<Face>& face, const std::shared_ptr<OuterVertex>& initPoint);

    // ----------- output and display functions ----------
    void exportVertices();
    void showPoly();

    // ----------- getters ----------
    int get_numberOfFeet() const;
    int get_numberOfAcceletations() const;

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
    static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const& vect);

private:
    Eigen::Vector3d const m_gravity;
    double m_mass;

    int m_numberOfFeet;
    std::vector<ContactPoints> m_feet;

    int m_numberOfAccelerations;
    std::vector<Eigen::Vector3d> m_accelerations;

    // attributes used for the LP problem
    glp_prob *m_lp;
    int m_numberOfFrictionSides; // Number of sides of the approximation of the friction cones
    Eigen::MatrixXd m_Q_u;
    Eigen::MatrixXd m_Q_c;
    Eigen::MatrixXd m_R_inv_T_b;

    int m_numberOfIterations;
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
};

#endif // ROBOT_H_INCLUDED
