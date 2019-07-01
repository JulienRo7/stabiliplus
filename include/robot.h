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
#include "math.h"

// libraries
#include "tinyxml2.h"
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

    static Eigen::Matrix3d skewSymmetric(Eigen::Vector3d const& vect);

    Eigen::MatrixXd computeMatrixA1();
    Eigen::MatrixXd computeMatrixA2(Eigen::Vector3d const& acceleration);
    Eigen::VectorXd computeVector_t(Eigen::Vector3d const& acceleration);

    Eigen::MatrixXd buildMatrixA();
    Eigen::VectorXd buildVectorB();
    Eigen::MatrixXd buildFrictionF();

    void buildStabilityProblem();
    Eigen::Vector3d solveStabilityProblem(Eigen::Vector3d const& direction);

    void projectionStabilityPolyhedron();

    Eigen::Vector3d computeInnerPoint();
    void buildInnerPoly();
    void updateInnerPoly(Vertex* newVertex, Face* dirFace);

    void exportVertices();

    int get_numberOfFeet();
    int get_numberOfAcceletations();

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

    // inner polyhedron
    std::vector<Vertex> m_vertices;
    Eigen::Vector3d m_innerPoint;

    std::vector<Edge> m_edges;
    std::vector<Face> m_faces;

};

#endif // ROBOT_H_INCLUDED
