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
#include <map>
#include <cmath>
#include <algorithm>

#include <chrono>

// libraries
#include <tinyxml2.h>
#include <Eigen/Dense>
#include <glpk.h>

// custom libraries
#include "contactPoints.h"
#include "vertex.h"
#include "edge.h"
#include "face.h"

class Robot
{

public:
    Robot();
    Robot(std::string const& robot_file_name, int numFrictionSides=8);
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
    void updateInnerPoly(Vertex* newVertex, Face* dirFace);
    void buildOuterPoly();

    // ----------- output and display functions ----------
    void exportVertices();
    void showPoly();


    // ----------- getters ----------
    int get_numberOfFeet();
    int get_numberOfAcceletations();

    int get_numberOfVertices();

    // ---------- static functions ---------
    static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const& vect);
    static void unionEdgeLists(std::list<Edge*> &listA, std::list<Edge*> &listB);
    static void unionFaceLists(std::list<Face*> &listA, std::list<Face*> &listB);

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
    std::vector<Vertex*> m_vertices;
    Eigen::Vector3d m_innerPoint;

    std::vector<Edge*> m_edges;
    std::vector<Face*> m_faces;

    // outer polyhedron
    std::vector<Vertex*> m_outerVertices;
    std::vector<Edge*> m_outerEdges;
    std::vector<Face*> m_outerFaces;
};

#endif // ROBOT_H_INCLUDED
