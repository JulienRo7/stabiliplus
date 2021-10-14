#include "experimenter.h"

using namespace std;

int main(int argc, char * argv[])
{
  int mode = 1;

  std::string file_robot = "./robots/robot_8.xml";
  int numFrictionSides = 16;

  Solver solver = GLPK;
  bool robust = true;

  for(int i = 0; i < argc; i++)
  {
    // std::cout << "|" << argv[i] << "|" << '\n';
    if(std::string(argv[i]) == "--mode")
    {
      mode = std::atoi(argv[i + 1]);
    }
    else if(std::string(argv[i]) == "--robot")
    {
      file_robot = std::string(argv[i + 1]);
    }
    else if(std::string(argv[i]) == "--fric_sides")
    {
      numFrictionSides = std::atoi(argv[i + 1]);
    }
    else if(std::string(argv[i]) == "--solver")
    {
      std::string solver_name = std::string(argv[i + 1]);

      if(solver_name == "GLPK")
      {
        solver = GLPK;
      }
      else if(solver_name == "LP_SOLVE")
      {
        solver = LP_SOLVE;
      }
      else if(solver_name == "GUROBI")
      {
        solver = GUROBI;
      }
      else
      {
        std::cout << "Unknown solver name " << solver_name << ". Using the default one (GLPK) instead." << std::endl;
      }
    }
    else if(std::string(argv[i]) == "--robust")
    {
      robust = argv[i + 1] == "TRUE";
    }
  }

  std::cout << "Mode: " << mode << '\n';
  std::cout << "Robot file: " << file_robot << '\n';
  std::cout << "Number of friction sides: " << numFrictionSides << '\n';
  std::cout << "Solver: " << (solver==GLPK ? "GLPK": (solver==LP_SOLVE ? "LP_SOLVE" : "GUROBI")) << '\n';
  std::cout << "Using robust stability: " << (robust ? "yes" : "no") << '\n';
  std::cout << '\n';

  /*
  mode 1 : generate one robot and compute its stability polyhedron
      results can be displayed using postprocess.py with python3
  
  mode 2 : computes the balance region for all contact ser in a folder

  Available robots have xml files in the robots folder

  only robot_2.xml has 4 accelerations
  */

  Experimenter experience(mode, file_robot, numFrictionSides, solver, robust);

  experience.run();

  experience.save();

  std::cout << '\n' << "Experiment Finnished! Run postprocess.py to display results" << '\n';

  return 0;
}
