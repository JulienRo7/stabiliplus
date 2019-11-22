#include "polytope/staticStabilityPolytope.h"

// StaticStabilityPolytope::StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration, double maxError, Solver solver):
//   m_contactSet(contactSet), m_solver(solver),
//   m_maxIterations(maxNumberOfIteration), m_iteration(0),
//   m_maxError(maxError), m_error(2*maxError),
//   m_initTime(0), m_LPTime(0), m_structTime(0)
// {
//   switch(solver)
//     {
//     case GLPK:
//       m_lp = new GlpkWrapper;
//       break;
      
//     case LP_SOLVE:
//       m_lp = new LPSolveWrapper;
//       break;

//     case GUROBI:
//       m_lp = new GurobiWrapper;
//       break;
//     }
// }

StaticStabilityPolytope::~StaticStabilityPolytope()
{
  
}

// ----- main class methods
void StaticStabilityPolytope::initSolver()
{
  auto start = std::chrono::high_resolution_clock::now();
  
  auto B = m_contactSet.buildStaticVectorB();
  auto A = m_contactSet.buildStaticMatrixA();
  auto F = m_contactSet.buildStaticFrictionF();
  auto f = m_contactSet.buildStaticFrictionVectorf();

  // std::cout << "Vector B: " << B.transpose() << std::endl;
  // std::cout << "Matrix A: " << A << std::endl;
  // std::cout << "Matrix F: " << F << std::endl;
  // std::cout << "Vector f: " << f.transpose() << std::endl;
  
  m_lp->buildProblem(B, A, F, f);

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop - start);
  m_initTime = duration.count();
}

void StaticStabilityPolytope::solveLP(Eigen::Vector2d const& direction, Eigen::Vector2d &vertex)
{
  auto start = std::chrono::high_resolution_clock::now();
  m_lp->set_staticSearchDirection(direction);

  m_lp->solveProblem();

  vertex = m_lp->get_staticResult();

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop - start);
  m_LPTime += duration.count();
}

void StaticStabilityPolytope::projectionStabilityPolyhedron()
{
  auto start = std::chrono::high_resolution_clock::now();
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
  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop-start);

  m_structTime = duration.count() - m_LPTime;
  

}

// ----- output -----
void StaticStabilityPolytope::writeToStream(std::ofstream& stream ) const
{
  if (stream)
    {
      auto it_pt = m_points.begin();
      auto pt = (*it_pt)->next();
      
      while (pt != *it_pt)
	{
	  pt->writeToStream(stream);
	  pt = pt->next();
	}
      pt->writeToStream(stream);
    }
  else
    {
      std::cout << "Error: Output stream not open." << std::endl;
    }  
}

std::vector<Eigen::Vector4d> StaticStabilityPolytope::constraintPlanes() const
{
  std::vector<Eigen::Vector4d> planes;

  for (auto pt: m_points)
    {
      planes.push_back(pt->plane());
    }
  
  return planes;
}

Eigen::Vector3d StaticStabilityPolytope::baryPoint() const
{
  Eigen::Vector2d bary = Eigen::Vector2d::Zero();

  for (auto pt: m_points)
    {
      bary += pt->innerVertex();
    }

  bary /= m_points.size();
  
  Eigen::Vector3d baryPt;
  baryPt << bary, 0;
  return baryPt;
}

void StaticStabilityPolytope::showPointsNeighbours()
{
  std::cout << "Neighborhood: " << std::endl;
  for (auto it_pt: m_points)
    {
      std::cout << it_pt->prec() << "->" << it_pt << "->" << it_pt->next() << std::endl;
    }
}

// ----- getters -----
