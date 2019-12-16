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
  void run_exp1();
  void run_exp1_static();
  void run_exp1_robust();
  void run_exp2();
  void run_exp2_static();
  void run_exp2_robust();
  void run_exp3();
  void run_exp3_static();
  void run_exp3_robust();

  // ---------- inputs and setters -----------

  // ---------- outputs and getters -----------
  void save();

private:
  std::shared_ptr<ContactSet> m_contactSetPtr;
  int m_mode;
  bool m_robust;
  int m_numFrictionSides;
  Solver m_solver;

  std::vector<std::shared_ptr<StabilityPolytope>> m_polytopes;
  std::vector<int> m_total_times_ms;

  std::string stabiliplus_path;
};
