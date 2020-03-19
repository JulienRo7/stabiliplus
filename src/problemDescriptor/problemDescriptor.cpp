#include "problemDescriptor/problemDescriptor.h"

ProblemDescriptor::ProblemDescriptor(std::string name) : m_name(name)
{
  //std::cout << "ProblemDescriptor constructor for object called " << name << std::endl;
}

ProblemDescriptor::~ProblemDescriptor()
{
  // std::cout << "ProblemDescriptor destructor called!" << '\n';
}

