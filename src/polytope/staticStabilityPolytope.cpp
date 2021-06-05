#include "polytope/staticStabilityPolytope.h"

// StaticStabilityPolytope::StaticStabilityPolytope(ContactSet contactSet, int maxNumberOfIteration, double maxError,
// Solver solver):
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
  // delete m_contactSetPtr;
  // m_contactSetPtr = nullptr;
}

// ----- main class methods
void StaticStabilityPolytope::initSolver()
{
  // m_contactSetPtr = static_cast<ContactSet *>(m_pdPtr);

  auto start = std::chrono::high_resolution_clock::now();

  m_pdPtr->update(); 
  //m_pdPtr->showContactSet();
  
  // std::cout<<"Inside StaticStabilityPolytope: "<<std::endl;

  // std::cout<<"A matrix is: "<<std::endl<<m_pdPtr->getMatrixA()<<std::endl;
  // std::cout<<"B vector is: "<<std::endl<<m_pdPtr->getVectorB().transpose()<<std::endl;

  // std::cout<<"F matrix is: "<<std::endl<<m_pdPtr->getFrictionF()<<std::endl;
  // std::cout<<"f vector is: "<<std::endl<<m_pdPtr->getFrictionVectorf().transpose()<<std::endl;

  m_lp->buildProblem(m_pdPtr->getVectorB(),  m_pdPtr->getMatrixA(), 
		    m_pdPtr->getFrictionF(), m_pdPtr->getFrictionVectorf());

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  m_initTime = duration.count();
}

void StaticStabilityPolytope::solveLP(Eigen::Vector2d const & direction, Eigen::Vector2d & vertex)
{
  auto start = std::chrono::high_resolution_clock::now();
  m_lp->set_staticSearchDirection(direction);

  m_lp->solveProblem();

  vertex = m_lp->get_staticResult();

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  m_LPTime += duration.count();
}

void StaticStabilityPolytope::projectionStabilityPolyhedron()
{
  if(!this->computeProjectionStabilityPolyhedron()){
    throw std::runtime_error("method failed!");
  }
}

bool StaticStabilityPolytope::computeProjectionStabilityPolyhedron()
{
  auto start = std::chrono::high_resolution_clock::now();
  // initialization of the algorithm
  Eigen::Vector2d dir, vertex;

  // inner vertices
  dir << 1.0, 0.0;
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p1(new StaticPoint(dir, vertex));
  m_points.push_back(p1);


  dir << cos(2 * M_PI / 3), sin(2 * M_PI / 3);
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p2(new StaticPoint(dir, vertex));
  m_points.push_back(p2);
  p1->next(p2);
  p2->prec(p1);

  dir << cos(4 * M_PI / 3), sin(4 * M_PI / 3);
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p3(new StaticPoint(dir, vertex));
  m_points.push_back(p3);
  p2->next(p3);
  p3->prec(p2);
  p3->next(p1);
  p1->prec(p3);

  // initialisation of the error
  std::for_each(m_points.begin(), m_points.end(), [](auto p){p->updateMeasure();});
  m_error = p1->measure() + p2->measure() + p3->measure();

  if(m_error==0){
    return false; //NOTE the constraints might correspond to an empty set?
    // if we have an empty set, it should be checked using the area of the initial inner region
    // and maybe use the area of the outer region as a double check
  }
  if(m_error<=0){
    return false; //TODO this should not happen or the set is not convex
  }
  //std::cout<<"Initial projection error: "<<m_error<<std::endl;

  while(!stopCriterion())
  {
    //std::cout << "##### Iteration " << m_iteration+1 << " #####" << std::endl;
    //std::cout << "Error " << m_error << " (m_maxError: " << m_maxError << ") \n";
    auto p_max = *(std::max_element(m_points.begin(), m_points.end(), **(m_points.begin())));
    auto p_next = p_max->next();

    // update the error
    m_error -= p_max->measure();

    dir = p_max->normal();
    solveLP(dir, vertex);
    std::shared_ptr<StaticPoint> p(new StaticPoint(dir, vertex));

    // !!! Warning: the order here is important in order to avoid issue when the new point is close to the old one
    p_next->prec(p);
    p->next(p_next);
    p->prec(p_max);
    p_max->next(p);

    m_points.push_back(p);
    
    p->updateMeasure();
    p_max->updateMeasure();
    

    m_error += p_max->measure();
    m_error += p->measure();
    if(m_error<0){
      return false; //TODO
    }
    
    // showPointsNeighbours();

    m_iteration++;

    //std::cout<<"iteration-"<<m_iteration<<": "<<m_error<< ", "<<std::endl;
  }
  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  m_structTime = duration.count() - m_LPTime;
  return true;
}

// ----- output -----
void StaticStabilityPolytope::writeToStream(std::ofstream & stream) const
{
  if(stream)
  {
    auto it_pt = m_points.begin();
    auto pt = (*it_pt)->next();

    while(pt != *it_pt)
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

  for(auto pt : m_points)
  {
    planes.push_back(pt->plane());
  }

  return planes;
}

Eigen::Vector3d StaticStabilityPolytope::baryPoint() const
{
  Eigen::Vector2d bary = Eigen::Vector2d::Zero();

  for(auto pt : m_points)
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
  for(auto it_pt : m_points)
  {
    std::cout << it_pt->prec() << "->" << it_pt << "->" << it_pt->next() << std::endl;
  }
}

// ----- getters -----
int StaticStabilityPolytope::get_numberOfVertices() const
{
  return m_points.size();
}

const std::vector<Eigen::Vector2d> StaticStabilityPolytope::getInnerVertices() const
{
  std::vector<Eigen::Vector2d> vertices;
  
  auto it_pt = m_points.begin();
  auto pt = (*it_pt)->next();
  
  while(pt != *it_pt)
    {
      vertices.push_back(pt->innerVertex());
      pt = pt->next();
    }
  vertices.push_back(pt->innerVertex());

  return vertices;
}

const std::vector<Eigen::Vector2d> StaticStabilityPolytope::getOuterVertices() const
{
  std::vector<Eigen::Vector2d> vertices;
  
  auto it_pt = m_points.begin();
  auto pt = (*it_pt)->next();
  
  while(pt != *it_pt)
    {
      vertices.push_back(pt->outerVertex());
      pt = pt->next();
    }
  vertices.push_back(pt->outerVertex());

  return vertices;
}
