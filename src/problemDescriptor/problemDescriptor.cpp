#include "problemDescriptor/problemDescriptor.h"

ProblemDescriptor::ProblemDescriptor(std::string name) : m_name(name) {}

ProblemDescriptor::~ProblemDescriptor()
{
  // std::cout << "ProblemDescriptor destructor called!" << '\n';
}

