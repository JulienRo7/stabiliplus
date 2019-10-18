#ifndef EXPERIMENTER_H_INCLUDED
#define EXPERIMENTER_H_INCLUDED

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <string>

#include <tinyxml2.h>

#include "contactSet.h"
#include "stabilityPolytope.h"
#include "robustStabilityPolytope.h"
#include "staticStabilityPolytope.h"

class Experimenter
{
 public:
  // ---------- constructors and destructor -----------
  Experimenter(int mode, std::string const& contact_set_file_name, int numFrictionSides=8, Solver solver = GLPK);
  ~Experimenter();

  // ---------- main functions -----------
  void run();
  void run_exp1();
  void run_exp2();
  void run_exp3();
  void run_exp4();
    
  // ---------- inputs and setters -----------

  // ---------- outputs and getters -----------
  void save();


 private:
  ContactSet m_contactSet;
  int m_mode;
  int m_numFrictionSides;
  Solver m_solver;

  std::vector<std::shared_ptr<StabilityPolytope>> m_polytopes;
  std::vector<int> m_total_times_ms;
  
  std::string stabiliplus_path;

};

#endif // EXPERIMENTER_H_INCLUDED
