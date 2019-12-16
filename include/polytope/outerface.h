#ifndef OUTERFACE_H_INCLUDED
#define OUTERFACE_H_INCLUDED

#include "vertex.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

// class OuterVertex; // Forward declaration

class OuterFace
{
public:
  // ---------- constructors ----------
  OuterFace(std::shared_ptr<Vertex> innerVertex);

  // ---------- destructor ----------
  ~OuterFace();

  // ---------- class functions ----------
  // static bool isFaceOf(OuterVertex* outerVertex);

  // ---------- getters ----------
  int get_index() const;
  std::shared_ptr<Vertex> get_innerVertex() const;
  Eigen::Vector3d get_normal() const;
  double get_offset() const;

  // ---------- setters ----------

private:
  static int GlobalOuterFaceCounter;
  int m_index;

  std::shared_ptr<Vertex> m_innerVertex; // inner vertex corresponding to that face
  // Eigen::Vector3d m_normal;
};

#endif // OUTERFACE_H_INCLUDED
