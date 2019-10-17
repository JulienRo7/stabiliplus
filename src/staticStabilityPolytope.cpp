#include "stabiliplus/staticStabilityPolytope.h"

StaticStabilityPolytope::StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration, Solver solver):
  m_contactSet(contactSet), m_solver(solver),
  m_maxIterations(maxNumberOfIteration), m_iteration(0)
{
  m_maxIterations = 3;
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
  m_innerVertices.push_back(vertex);
  m_searchDirections.push_back(dir);

  dir << cos(2*M_PI/3), sin(2*M_PI/3);
  solveLP(dir, vertex);
  m_innerVertices.push_back(vertex);
  m_searchDirections.push_back(dir);

  dir << cos(4*M_PI/3), sin(4*M_PI/3);
  solveLP(dir, vertex);
  m_innerVertices.push_back(vertex);
  m_searchDirections.push_back(dir);

  // outer vertices
  auto it_vert1 = m_innerVertices.begin();
  auto it_dir1 = m_searchDirections.begin();
  auto it_vert2 = m_innerVertices.begin();
  it_vert2++;
  auto it_dir2 = m_searchDirections.begin();
  it_dir2++;

  m_outerVertices.push_back(computeOuterVertex(*it_vert1, *it_dir1, *it_vert2, *it_dir2));
  m_normals.push_back(computeSidesNormal(*it_vert1, *it_vert2));

  it_vert1 ++;
  it_vert2 ++;
  it_dir1 ++;
  it_dir2 ++;
  m_outerVertices.push_back(computeOuterVertex(*it_vert1, *it_dir1, *it_vert2, *it_dir2));
  m_normals.push_back(computeSidesNormal(*it_vert1, *it_vert2));

  it_vert1 ++;
  it_vert2 = m_innerVertices.begin();
  it_dir1 ++;
  it_dir2 = m_searchDirections.begin();
  m_outerVertices.push_back(computeOuterVertex(*it_vert1, *it_dir1, *it_vert2, *it_dir2));
  m_normals.push_back(computeSidesNormal(*it_vert1, *it_vert2));

  while (m_iteration < m_maxIterations)
    {
      
      m_iteration++;
    }
}

Eigen::Vector2d StaticStabilityPolytope::computeOuterVertex(const Eigen::Vector2d& v1, const Eigen::Vector2d& d1, const Eigen::Vector2d& v2, const Eigen::Vector2d& d2)
{
  double a(d1(0)), b(d1(1)), c(d2(0)), d(d2(1));
  double det = a*d - b*c;
  double off1(v1.transpose()*d1), off2(v2.transpose()*d2);

  Eigen::Vector2d oV;
  
  oV << d*off1-b*off2, a*off2-c*off1;
  oV/=det;

  return oV;
}

Eigen::Vector2d StaticStabilityPolytope::computeSidesNormal(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
  Eigen::Vector2d n;
  n << -v1(1)+v2(1), v1(0)-v2(0);
  return n/n.norm();
}


// ----- output -----
void StaticStabilityPolytope::saveResults(std::string file_name )
{
  std::cout << "Saving results to " << file_name << std::endl;
  std::ofstream file_stream(file_name);
  if (file_stream)
    {
      for (auto v: m_innerVertices)
	{
	  file_stream << "iv;" // iv = inner vertice
		      << v(0) << ";"
		      << v(1) << ";"
		      << std::endl;
	}
      
      for (auto d: m_searchDirections )
	{
	  file_stream << "sd;" // sd = searchDirection
		      << d(0) << ";"
		      << d(1) << ";"
		      << std::endl;
	}
      

      for (auto v: m_outerVertices)
	{
	  file_stream << "ov;" // ov = outer vertice
		      << v(0) << ";"
		      << v(1) << ";"
		      << std::endl;
	}
      
      for (auto n: m_normals)
	{
	  file_stream << "no;" // no = normal
		      << n(0) << ";"
		      << n(1) << ";"
		      << std::endl;
	}
      
    }
  else
    {
      std::cout << "Could not open " << file_name << "for writing static stability results" << std::endl;
    }
  
  
  
}
