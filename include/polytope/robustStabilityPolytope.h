#ifndef ROBUST_STABILITY_POLYTOPE_H_INCLUDE
#define ROBUST_STABILITY_POLYTOPE_H_INCLUDE

// standard libraries
#include <list>
#include <map>

// libraries
#include <Eigen/Dense>

// custom libraries
#include "polytope/stabilityPolytope.h"

#include "polytope/vertex.h"
#include "polytope/edge.h"
#include "polytope/face.h"
#include "polytope/outervertex.h"
#include "polytope/outeredge.h"
#include "polytope/outerface.h"


class RobustStabilityPolytope: public StabilityPolytope
{
public:
  // ----------- constructors and destructor ----------
  using StabilityPolytope::StabilityPolytope; // inheriting constructors
  ~RobustStabilityPolytope();

  // ----------- main class methods ----------
  void initSolver(); // compute the reduced stability problem
  void solveLP(Eigen::Vector3d const& direction, Eigen::Vector3d &vertex);

  void projectionStabilityPolyhedron();

  Eigen::Vector3d computeInnerPoint();
  void buildInnerPoly();
  void updateInnerPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);
  void buildOuterPoly();
  void updateOuterPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace);

  void updateSupportFunctions(std::shared_ptr<Face>& dirFace);
  bool computeSupportFunction(std::shared_ptr<Face>& face, const std::shared_ptr<OuterVertex>& initPoint);

  double computeResidualFromScratch();

  // ----------- output and display functions ----------
  void writeToStream(std::ofstream& stream) const;
  void showPoly() const;
  std::vector<Eigen::Vector4d> constraintPlanes() const;
  Eigen::Vector3d baryPoint() const;

  // ----------- getters ----------
  int get_numberOfVertices() const;
  int get_numberOfFaces() const;
  int get_numberOfOuterVertices() const;
  int get_numberOfOuterFaces() const;

  double get_innerConvexMicro() const;
  double get_outerConvexMicro() const;
  double get_supportFunctionMicro() const;

  std::vector<Eigen::Vector3d> get_innerFaceNormals() const;
  std::vector<double> get_innerFaceOffsets() const;

  // ----------- setters ----------

  // ---------- static functions ---------

 private:
  
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

  // time measures
  double m_innerConvexMicro;
  double m_outerConvexMicro;
  double m_supportFunctionMicro;


};

#endif // ROBUST_STABILITY_POLYTOPE_H_INCLUDE
