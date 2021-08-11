#ifndef ROBUST_STABILITY_POLYTOPE_H_INCLUDE
#define ROBUST_STABILITY_POLYTOPE_H_INCLUDE

// standard libraries
#include <list>
#include <map>

// libraries
#include <Eigen/Dense>

// custom libraries
#include "polytope/edge.h"
#include "polytope/face.h"
#include "polytope/outeredge.h"
#include "polytope/outerface.h"
#include "polytope/outervertex.h"
#include "polytope/stabilityPolytope.h"
#include "polytope/vertex.h"

class RobustStabilityPolytope : public StabilityPolytope
{
public:
  // ----------- constructors and destructor ----------
  using StabilityPolytope::StabilityPolytope; // inheriting constructors
  ~RobustStabilityPolytope();

  // ----------- main class methods ----------
  void initSolver() override; // compute the reduced stability problem
  bool solveLP(Eigen::Vector3d const & direction, Eigen::Vector3d & vertex);

  void projectionStabilityPolyhedron() override;
  bool computeProjectionStabilityPolyhedron();

  Eigen::Vector3d computeInnerPoint();
  void buildInnerPoly();
  bool updateInnerPoly(std::shared_ptr<Vertex> & newVertex, std::shared_ptr<Face> & dirFace);
  void buildOuterPoly();
  bool updateOuterPoly(std::shared_ptr<Vertex> & newVertex, std::shared_ptr<Face> & dirFace);

  void updateSupportFunctions(std::shared_ptr<Face> & dirFace);
  bool computeSupportFunction(std::shared_ptr<Face> & face, const std::shared_ptr<OuterVertex> & initPoint);

  double computeResidualFromScratch();

  // ----------- output and display functions ----------
  void writeToStream(std::ofstream & stream) const override;
  tinyxml2::XMLElement * xmlPolytope(tinyxml2::XMLDocument & doc) const override;
  void showPoly() const;
  void computeHrep(Eigen::MatrixXd & Aineq, Eigen::VectorXd & bineq) const override;
  std::vector<Eigen::Vector4d> constraintPlanes() const override;
  std::vector<Eigen::Vector3d> vertices() const override;
  
  Eigen::Vector3d baryPoint() const override;

  // ----------- getters ----------
  int get_numberOfVertices() const;
  int get_numberOfEdges() const;
  int get_numberOfFaces() const;
  int get_numberOfOuterVertices() const;
  int get_numberOfOuterFaces() const;

  double get_innerConvexMicro() const;
  double get_outerConvexMicro() const;
  double get_supportFunctionMicro() const;

  std::vector<Eigen::Vector3d> get_innerFaceNormals() const;
  std::vector<double> get_innerFaceOffsets() const;
  const std::vector<Eigen::Vector3d> getInnerVertices() const;
  const void getRandomFeasiblePoint(Eigen::Vector3d & point) const;
  const bool getUniformRandomFeasiblePoint(Eigen::Vector3d & point, const int maxSamplingAttempts = 100) const;
  const bool isPointFeasible(Eigen::Vector3d & point) const;

  Eigen::Vector3d chebichevCenter() const override;
  
  /*
  inline ProblemDescriptor * problemDescriptor() override
  {
    return m_contactSetPtr;
  }

  */


  // this seems to be a bit dangerous: now every body can modify the verties and the faces...
  inline std::vector<std::shared_ptr<Vertex>> fullVertices() const
  {
    return m_vertices;
  }

  inline std::vector<std::shared_ptr<Face>> fullFaces() const
  {
    return m_faces;
  }
  
  // ----------- setters ----------

  // ---------- static functions ---------



private:
  // ContactSet * m_contactSetPtr;

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
  double m_innerConvexMicro=0;
  double m_outerConvexMicro=0;
  double m_supportFunctionMicro=0;
  const int projectedPolytopeDim = 3;
};

#endif // ROBUST_STABILITY_POLYTOPE_H_INCLUDE
