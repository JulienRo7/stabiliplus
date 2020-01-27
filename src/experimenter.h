#pragma once

#include "polytope/robustStabilityPolytope.h"
#include "polytope/stabilityPolytope.h"
#include "polytope/staticStabilityPolytope.h"
#include "problemDescriptor/contactSet.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tinyxml2.h>
#include <typeinfo> // operator typeid
#include <vector>

class Experimenter
{
public:
  // ---------- constructors and destructor -----------
  Experimenter(int mode,
               std::string const & contact_set_file_name,
               int numFrictionSides = 8,
               Solver solver = GLPK,
               bool robust = true);
  ~Experimenter();

  // ---------- main functions -----------
  void run();
  /*! \brief Experiment 1 compute one equilibrium region for the given parameters
   */
  void run_exp1();

  /*! \brief Experiment 2 compares the computing times for the different LP solvers
   *
   * \note to work properly, this experiment requires LP_solve and gurobi to be installed
   */
  void run_exp2();

  /*! \brief Experiment 3 create an animation for a contact set with a moving contact
   * 
   */
  void run_exp3();

  /*! \brief Experiment 4 dislay what happens when the limit force on a contact point changes
   * 
   */
  void run_exp4();
  
  // ---------- inputs and setters -----------

  // ---------- outputs and getters -----------
  void save();

private:
  int m_mode;
  bool m_robust;
  int m_numFrictionSides;
  Solver m_solver;
  std::string m_contactSetFileName;
  
  std::vector<std::shared_ptr<ContactSet>> m_contactSets;
  std::vector<std::shared_ptr<StabilityPolytope>> m_polytopes;
  std::vector<int> m_total_times_ms;
};
