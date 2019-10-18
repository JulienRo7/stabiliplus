#include "stabiliplus/staticStabilityPolytope.h"

StaticStabilityPolytope::StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration, double maxError, Solver solver):
  m_contactSet(contactSet), m_solver(solver),
  m_maxIterations(maxNumberOfIteration), m_iteration(0),
  m_maxError(maxError), m_error(2*maxError)
{
  switch(solver)
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

StaticStabilityPolytope::~StaticStabilityPolytope()
{
  delete m_lp;
}

// ----- main class methods
void StaticStabilityPolytope::initSolver()
{
  auto B = m_contactSet.buildStaticVectorB();
  auto A = m_contactSet.buildStaticMatrixA();
  auto F = m_contactSet.buildStaticFrictionF();
  auto f = m_contactSet.buildStaticFrictionVectorf();

  // std::cout << "Vector B: " << B.transpose() << std::endl;
  // std::cout << "Matrix A: " << A << std::endl;
  // std::cout << "Matrix F: " << F << std::endl;
  // std::cout << "Vector f: " << f.transpose() << std::endl;
  
  
  m_lp->buildProblem(B, A, F, f);
}

void StaticStabilityPolytope::solveLP(Eigen::Vector2d const& direction, Eigen::Vector2d &vertex)
{
  m_lp->set_staticSearchDirection(direction);

  m_lp->solveProblem();

  vertex = m_lp->get_staticResult();
}

void StaticStabilityPolytope::projectionStabilityPolyhedron()
{
  // initialization of the algorithm
  Eigen::Vector2d dir, vertex;

  // inner vertices 
  dir << 1, 0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p1 (new StaticPoint(dir, vertex));
  m_points.push_back(p1);
  
  dir << cos(2*M_PI/3), sin(2*M_PI/3);
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p2 (new StaticPoint(dir, vertex));
  m_points.push_back(p2);
  p1->next(p2);
  p2->prec(p1);

  dir << cos(4*M_PI/3), sin(4*M_PI/3);
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p3 (new StaticPoint(dir, vertex));
  m_points.push_back(p3);
  p2->next(p3);
  p3->prec(p2);
  p3->next(p1);
  p1->prec(p3);

  // initialisation of the error
  m_error = p1->measure() + p2->measure() + p3->measure();

  
  while (!stopCriterion())
    {
      // std::cout << "##### Iteration " << m_iteration+1 << " #####" << std::endl;
      // std::cout << "Error " << m_error << " (m_maxError: " << m_maxError << ") \n";
      auto p_max = std::max_element(m_points.begin(), m_points.end(), **(m_points.begin()));
      auto p_next = (*p_max)->next();

      // update the error
      m_error -= (*p_max)->measure();
      
      dir = (*p_max)->normal();
      solveLP(dir, vertex);
      std::shared_ptr<StaticPoint> p (new StaticPoint(dir, vertex));

      // !!! Warning: the order here is important in order to avoid issue when the new point is close to the old one 
      p_next->prec(p);
      p->next(p_next);
      p->prec(*p_max);
      (*p_max)->next(p);
      
      m_error += (*p_max)->measure();
      m_error += p->measure();
      
      m_points.push_back(p);
      // showPointsNeighbours();

      m_iteration++;
    }
  

}

bool StaticStabilityPolytope::stopCriterion() const // return true when the algorithm has to stop
{
  return (m_iteration > m_maxIterations || m_error < m_maxError);
}


// ----- output -----
void StaticStabilityPolytope::saveResults(std::string file_name )
{
  std::cout << "Saving results to " << file_name << std::endl;
  std::ofstream file_stream(file_name);
  if (file_stream)
    {
      auto it_pt = m_points.begin();
      auto pt = (*it_pt)->next();
      
      while (pt != *it_pt)
	{
	  pt->writeToStream(file_stream);
	  pt = pt->next();
	}
      pt->writeToStream(file_stream);
    }
  else
    {
      std::cout << "Could not open " << file_name << "for writing static stability results" << std::endl;
    }  
}

void StaticStabilityPolytope::showPointsNeighbours()
{
  std::cout << "Neighborhood: " << std::endl;
  for (auto it_pt: m_points)
    {
      std::cout << it_pt->prec() << "->" << it_pt << "->" << it_pt->next() << std::endl;
    }
}