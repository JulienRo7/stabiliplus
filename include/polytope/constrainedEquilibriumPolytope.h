#ifndef CONSTRAINED_EQUILIBRIUM_POLYTOPE_H_INCLUDE
#define CONSTRAINED_EQUILIBRIUM_POLYTOPE_H_INCLUDE

// standard libraries
#include <map>
#include <utility>
// libraries

// custom libraries
#include "polytope/stabilityPolytope.h"
#include "polytope/robustStabilityPolytope.h"

#include "problemDescriptor/contactSet.h"

// qhull
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullFacetSet.h>

class ConstrainedEquilibriumPolytope : public StabilityPolytope
{
 public:
  //constructors
  ConstrainedEquilibriumPolytope(std::shared_ptr<ContactSet> contactSet, int maxIteration, double maxError, Solver solverType);

  // main functions
  void initSolver() override;
  void projectionStabilityPolyhedron() override;
  void endSolver() override;
  void writeToStream(std::ofstream & stream) const override;
  tinyxml2::XMLElement * xmlPolytope(tinyxml2::XMLDocument & doc) const override;
  void save(std::string name) const;
  
  std::vector<Eigen::Vector4d> constraintPlanes() const override;
  std::vector<Eigen::Vector3d> vertices() const override;

  bool vertexInPlanes(Eigen::Vector3d vertex, std::vector<Eigen::Vector4d> planes, double eps=0.000001) const;
  
  Eigen::Vector3d baryPoint() const override;
  
  inline Eigen::Vector3d baryPointMin() const
  {
    return polyMin_->baryPoint();
  }
  
  inline Eigen::Vector3d baryPointMax() const
  {
    return polyMax_->baryPoint();
  }
  
  Eigen::Vector3d chebichevCenter() const override;
  
  inline Eigen::Vector3d chebichevCenterMin() const
  {
    return polyMin_->chebichevCenter();
  }
  
  inline Eigen::Vector3d chebichevCenterMax() const
  {
    return polyMax_->chebichevCenter();
  }

  int get_numberOfVertices() const override;

  // Eigen::Vector3d projectChebMaxOnPolyMin() const;
  inline std::shared_ptr<RobustStabilityPolytope> polyMin() const
  {
    return polyMin_;
  }
  
 private:
  std::vector<std::string> constrainedContactNames_;
  std::vector<double> constrainedContactFmax_;
  std::vector<double> constrainedContactFmin_;

  std::shared_ptr<ContactSet> contactSetMax_;
  std::shared_ptr<ContactSet> contactSetMin_;
  
  std::shared_ptr<RobustStabilityPolytope> polyMax_;
  std::shared_ptr<RobustStabilityPolytope> polyMin_;

  std::map<int, Eigen::Vector3d> vertices_;

  
  /* pairs of indexes correspodong to the edges
   */
  std::vector<std::pair<int, int>> edges_;
  
  std::vector<Eigen::Vector4d> planes_;
};

#endif // CONSTRAINED_EQUILIBRIUM_POLYTOPE_H_INCLUDE
