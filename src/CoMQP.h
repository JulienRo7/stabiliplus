#pragma once

/* #include <mc_rbdyn/Robot.h> */
/* #include <mc_rtc/Configuration.h> */
/* #include <mc_rtc/gui.h> */
/* #include <mc_rtc/log/Logger.h> */

#include <problemDescriptor/contactSet.h>

#include <Eigen/Dense>
#include <eigen-quadprog/QuadProg.h>

class CoMQP
{
public:
  CoMQP();
  CoMQP(std::shared_ptr<ContactSet> contactSet);

  void setContactSet(std::shared_ptr<ContactSet> contactSet);
  void addCoMRegionPlanes(std::vector<Eigen::Vector4d> planes);

  void updateProblem();
  bool solve(Eigen::Vector3d currentCoM);
  bool solveNonConstrained(Eigen::Vector3d currentCoM);
  bool solveConstrained(Eigen::Vector3d currentCoM);

  const Eigen::VectorXd & resultVector() const;
  Eigen::Vector3d resultCoM() const;

  int errorCode() const;

  inline void dontConsiderConstrained()
  {
    considerConstrainedContacts_ = false;
  }

  inline void considerConstrained()
  {
    considerConstrainedContacts_ = true;
  }
  
 private:
  std::shared_ptr<ContactSet> contactSet_;
  Eigen::QuadProgDense solver_;

  std::vector<Eigen::Vector4d> comRegionPlanes_;
  bool considerConstrainedContacts_;
};
