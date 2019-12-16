#ifndef OUTERVETEX_H_INCLUDED
#define OUTERVETEX_H_INCLUDED

#include "outerface.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

class OuterEdge; // Forward declaration

class OuterVertex : public std::enable_shared_from_this<OuterVertex>
{
public:
  // ---------- constructors ----------
  OuterVertex(std::shared_ptr<OuterFace> face1, std::shared_ptr<OuterFace> face2, std::shared_ptr<OuterFace> face3);
  OuterVertex(Eigen::Vector3d coordinates);

  // ---------- destructor ----------
  ~OuterVertex();

  // ---------- class methods ----------
  void computeCoordinates();
  bool strictlyContainedInHalfspace(std::shared_ptr<OuterFace> outerFace, double const eps = 0.0) const;

  std::vector<std::shared_ptr<OuterVertex>> findNeighbors();
  // ---------- getters ----------
  int get_index() const;
  Eigen::Vector3d get_coordinates() const;
  std::vector<std::shared_ptr<OuterFace>> get_outerFaces() const;
  std::vector<std::shared_ptr<OuterEdge>> get_outerEdges() const;

  // ---------- setters ----------
  void add_outerEdge(std::shared_ptr<OuterEdge> outerEgde);
  void remove_outerEdge(std::shared_ptr<OuterEdge> outerEdge);
  void add_outerFace(std::shared_ptr<OuterFace> outerFace);

private:
  static int GlobalOuterVertexCounter;
  int m_index;

  Eigen::Vector3d m_coordinates;

  std::vector<std::shared_ptr<OuterFace>> m_outerFaces;
  std::vector<std::shared_ptr<OuterEdge>> m_outerEdges;
};

#endif // OUTERVETEX_H_INCLUDED
