#ifndef EXPERIMENTER_H_INCLUDED
#define EXPERIMENTER_H_INCLUDED

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <memory>

#include <tinyxml2.h>
#include <ros/package.h>

#include "robot.h"
#include "stability_polytope.h"

class Experimenter
{
public:
    // ---------- constructors and destructor -----------
    Experimenter(int mode, std::string const& robot_file_name, int numFrictionSides=8);
    ~Experimenter();

    // ---------- main functions -----------
    void run();
    void run_exp1();
    void run_exp2();
    void run_exp3();

    // ---------- inputs and setters -----------

    // ---------- outputs and getters -----------
    void save();


private:
    Robot m_robot;
    std::vector<std::shared_ptr<StabilityPolytope>> m_polytopes;
    int m_mode;
    std::string stabiliplus_path;

};

#endif // EXPERIMENTER_H_INCLUDED