#ifndef FACE_H_INCLUDED
#define FACE_H_INCLUDED

#include <iostream>
#include <vector>
#include <memory>

#include "vertex.h"
#include "outervertex.h"

#include <Eigen/Dense>

class Edge; // Forward declaration

class Face: public std::enable_shared_from_this<Face>
{
public:
    Face(std::shared_ptr<Vertex> vertex1, std::shared_ptr<Vertex> vertex2, std::shared_ptr<Vertex> vertex3, std::shared_ptr<Edge> edge1, std::shared_ptr<Edge> edge2, std::shared_ptr<Edge> edge3, Eigen::Vector3d innerPoint);
    ~Face();

    void init();
    void finish();

    bool isSame(Face const& b) const;
    /*
    Note: it is considered that the normal is toward the outside
    */
    bool pointInHalfSpace(Eigen::Vector3d const& point, double const eps = 0.0) const;

    std::vector<std::shared_ptr<Face>> findNeighbors();

    // ---------- getters ----------
    int get_index() const;
    double get_area() const;
    double get_offset() const;

    Eigen::Vector3d get_normal() const;

    std::shared_ptr<Edge> get_edge1() const;
    std::shared_ptr<Edge> get_edge2() const;
    std::shared_ptr<Edge> get_edge3() const;
    std::vector<std::shared_ptr<Edge>> get_edges() const;

    std::shared_ptr<Vertex> get_vertex1() const;
    std::shared_ptr<Vertex> get_vertex2() const;
    std::shared_ptr<Vertex> get_vertex3() const;

    std::shared_ptr<OuterVertex> get_supportPoint() const;
    double get_supportFunction() const;

    double get_measure() const;

    // ---------- setters ----------
    void set_area_null();
    void set_supportPoint(std::shared_ptr<OuterVertex> supportPoint);

    // ---------- static functions ----------
    static bool compareFacesMeasure(std::shared_ptr<Face> faceA, std::shared_ptr<Face> faceB);

private:

    static int GlobalFaceCounter;
    int m_index;

    Eigen::Vector3d m_normal;
    double m_offset;
    double m_area;

    std::shared_ptr<Vertex> m_vertex1;
    std::shared_ptr<Vertex> m_vertex2;
    std::shared_ptr<Vertex> m_vertex3;

    std::shared_ptr<Edge> m_edge1;
    std::shared_ptr<Edge> m_edge2;
    std::shared_ptr<Edge> m_edge3;

    std::shared_ptr<OuterVertex> m_supportPoint;
    double m_supportFunction;

};

// ----------- operators overloading ----------
bool operator==(Face const& a, Face const& b);

#endif // FACE_H_INCLUDED
