#ifndef STATIC_STABILITY_POLYTOPE_H_INCLUDE
#define STATIC_STABILITY_POLYTOPE_H_INCLUDE

// standart libraries

// libraries

// custom libraries
#include "polytope/stabilityPolytope.h"
#include "polytope/staticPoint.h"

class StaticStabilityPolytope: public StabilityPolytope
{
 public:
  // StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration = 50, double maxError = 1,  Solver solver = GLPK);
  using StabilityPolytope::StabilityPolytope; // inherite the constructor
  ~StaticStabilityPolytope();

  // ----- main class methods ------
  void initSolver();
  void solveLP(Eigen::Vector2d const& direction, Eigen::Vector2d &vertex);

  void projectionStabilityPolyhedron();

  // ----- output -----
  void writeToStream(std::ofstream& stream) const;
  std::vector<Eigen::Vector4d> constraintPlanes() const;
  Eigen::Vector3d baryPoint() const;

  void showPointsNeighbours();
  // ----- setters -----

  // ----- getters -----
  int get_numberOfVertices() const;
  
 private:
  // algorithm storage
  std::vector<std::shared_ptr<StaticPoint>> m_points;

};
#endif // STATIC_STABILITY_POLYTOPE_H_INCLUDE
