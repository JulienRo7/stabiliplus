#include "stabiliplus/stabilityPolytope.h"

StabilityPolytope::StabilityPolytope(ContactSet contactSet, int maxIteration, double maxError, Solver solverType):
  m_contactSet(contactSet), m_solverType(solverType),
  m_iteration(0), m_maxIteration(maxIteration),
  m_error(1000), m_maxError(maxError),
  m_LPTime(0), m_initTime(0), m_structTime(0)
{
  switch(m_solverType)
  {
    case GLPK:
      m_lp = new GlpkWrapper;
      break;

    case LP_SOLVE:
      m_lp = new LPSolveWrapper;
      break;

    case GUROBI:
      m_lp = new GurobiWrapper;
      break;
  }
}


StabilityPolytope::~StabilityPolytope()
{
  delete m_lp;
}


// ----------- main class methods ----------

bool StabilityPolytope::stopCriterion() const// return true when the algorithm must stop
{
  return (m_iteration >= m_maxIteration) || (m_error <= m_maxError);
}

// ----------- output and display functions ----------

// ----------- getters ----------

double StabilityPolytope::LPTime() const
{
    return m_LPTime;
}

double StabilityPolytope::initTime() const
{
    return m_initTime;
}

double StabilityPolytope::structTime() const
{
    return m_structTime;
}

ContactSet* StabilityPolytope::contactSet()
{
    return &m_contactSet;
}

Solver StabilityPolytope::solverType() const
{
  return m_solverType;
}
// ------------------ setter -----------------------

void StabilityPolytope::maxIteration(int maxIteration)
{
    m_maxIteration = maxIteration;
}
