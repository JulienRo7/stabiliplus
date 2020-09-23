#pragma once

#include <memory>
#include <utility>
//#include <math.h>

// stabiliplus
#include <polytope/robustStabilityPolytope.h>

// sch
#include <sch-core/S_Point.h>
#include <sch-core/S_Polyhedron.h>
#include <sch/CD/CD_Pair.h>

class PointProjector
{
 public:
  PointProjector();

  void setPolytope(std::shared_ptr<RobustStabilityPolytope> poly);
  void setPoint(Eigen::Vector3d point);
  void project();
  
  Eigen::Vector3d projectedPoint() const;
  double distance() const;

  inline bool isSet() const
  {
    return isSet_;
  }

  void displaySqueleton();
 private:
  std::shared_ptr<RobustStabilityPolytope> polytope_;
  sch::S_Polyhedron polyhedron_;
  sch::S_Point point_;

  double distance_;
  Eigen::Vector3d projectedPoint_;
  bool isSet_;
};
