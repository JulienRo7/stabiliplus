#ifndef STATIC_STABILITY_POLYTOPE_H_INCLUDE
#define STATIC_STABILITY_POLYTOPE_H_INCLUDE

// standart libraries

// libraries

// custom libraries
#include "polytope/stabilityPolytope.h"
#include "polytope/staticPoint.h"

class StaticStabilityPolytope : public StabilityPolytope
{
public:
  // StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration = 50, double maxError = 1,  Solver solver =
  // GLPK);
  using StabilityPolytope::StabilityPolytope; // inherite the constructor
  ~StaticStabilityPolytope();

  // ----- main class methods ------
  void initSolver() override;
  void solveLP(Eigen::Vector2d const & direction, Eigen::Vector2d & vertex);

  void projectionStabilityPolyhedron() override;

  // ----- output -----
  void writeToStream(std::ofstream & stream) const override;
  tinyxml2::XMLElement * xmlPolytope(tinyxml2::XMLDocument & doc) const override;
  void showPoly() const;
  std::vector<Eigen::Vector4d> constraintPlanes() const override;
  std::vector<Eigen::Vector3d> vertices() const override;
  
  Eigen::Vector3d baryPoint() const override;

  void showPointsNeighbours();
  // ----- setters -----

  // ----- getters -----

  int get_numberOfVertices() const;
  const std::vector<Eigen::Vector2d> getInnerVertices() const;
  
  /*
  inline std::shared_ptr<ProblemDescriptor>* problemDescriptor() override
  {
    return m_contactSetPtr;
  }

  */
  

  
private:
  // algorithm storage

  // std::shared_ptr<ContactSet>  m_contactSetPtr; // for now the contact set should not change
  std::vector<std::shared_ptr<StaticPoint>> m_points;
};
#endif // STATIC_STABILITY_POLYTOPE_H_INCLUDE
