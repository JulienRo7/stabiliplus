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
  
  // In the static case these matrices are not the right ones-> need to remove the columns and lines corresponding to z?
  
  auto A = m_pdPtr->getMatrixA(); 
  auto b = m_pdPtr->getVectorB();  
  auto G = m_pdPtr->getFrictionF();
  auto h = m_pdPtr->getFrictionVectorf();

  // std::cout<<"Inside StaticStabilityPolytope: "<<std::endl;
    
  // std::cout<<"A matrix is: "<<std::endl<< A <<std::endl;
  // std::cout<<"B vector is: "<<std::endl<< b.transpose() <<std::endl;

  // std::cout<<"F matrix is: "<<std::endl<< G <<std::endl;
  // std::cout<<"f vector is: "<<std::endl<< h.transpose() <<std::endl;
  
  m_lp->buildProblem(b, A, G, h); // #NotationConsistency

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  m_initTime = duration.count();
}

bool StaticStabilityPolytope::solveLP(Eigen::Vector2d const & direction, Eigen::Vector2d & vertex)
{
  auto start = std::chrono::high_resolution_clock::now();
  m_lp->set_staticSearchDirection(direction);

  bool ret = m_lp->solveProblem();

  vertex = m_lp->get_staticResult();

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  m_LPTime += duration.count();

  return ret;
}

void StaticStabilityPolytope::projectionStabilityPolyhedron()
{
  if(!this->computeProjectionStabilityPolyhedron()){
    saveToFile("/tmp/stabiliplus_fail_polytope.xml");
    throw std::runtime_error("Stabiliplus static projection failed: " + std::to_string(errorCode_));
  }
}

bool StaticStabilityPolytope::computeProjectionStabilityPolyhedron()
{
  auto start = std::chrono::high_resolution_clock::now();
  // initialization of the algorithm
  Eigen::Vector2d dir, vertex;

  // add some randomness to help avoiding some case (hopefully)...
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 2*M_PI);
  double theta = dis(gen);

  // inner vertices
  dir << cos(theta), sin(theta);
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p1 = std::make_shared<StaticPoint>(dir, vertex);
  m_points.push_back(p1);


  dir << cos(2 * M_PI / 3 + theta), sin(2 * M_PI / 3 + theta);
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p2 = std::make_shared<StaticPoint>(dir, vertex);
  m_points.push_back(p2);
  p1->next(p2);
  p2->prec(p1);

  dir << cos(4 * M_PI / 3 + theta), sin(4 * M_PI / 3 + theta);
  vertex<< 0.0 , 0.0;
  solveLP(dir, vertex);
  std::shared_ptr<StaticPoint> p3 = std::make_shared<StaticPoint>(dir, vertex);
  m_points.push_back(p3);
  p2->next(p3);
  p3->prec(p2);
  p3->next(p1);
  p1->prec(p3);

  // initialisation of the error
  std::for_each(m_points.begin(), m_points.end(), [](auto p){p->updateMeasure();});
  m_error = p1->measure() + p2->measure() + p3->measure();

  if(m_error==0){
    errorCode_ = 1;
    return false; //NOTE the constraints might correspond to an empty set?
    // if we have an empty set, it should be checked using the area of the initial inner region
    // and maybe use the area of the outer region as a double check
  }
  if(m_error<-1e-10){
    errorCode_ = 2;
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
    if(m_error<-1e-10){
      errorCode_ = 3;
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

tinyxml2::XMLElement * StaticStabilityPolytope::xmlPolytope(tinyxml2::XMLDocument & doc) const
{
  auto xmlPoly = doc.NewElement("polytope");
  xmlPoly->SetAttribute("type", "static");

  auto it_pt = m_points.begin();
  auto pt = (*it_pt)->next();
  
  // only insert the point if the normal is  not too close
  Eigen::Vector2d prevNormal = (*it_pt)->normal();
  double thres = 5e-2;
  while(pt != *it_pt)
    {
      auto normal = pt->normal();
      double dist = (normal-prevNormal).norm();
      if (dist >= thres)
	{
	  xmlPoly->InsertEndChild(pt->xmlStaticPoint(doc));
	  prevNormal = normal;
	}
      pt = pt->next();
    }
  xmlPoly->InsertEndChild(pt->xmlStaticPoint(doc));
  
  return xmlPoly;
  
}

void StaticStabilityPolytope::showPoly() const
{
  int cpt = 0;
  auto showPt = [&cpt](std::shared_ptr<StaticPoint> pt){
    std::cout << "  Vertex " << std::to_string(cpt) << std::endl;
    std::cout << "    Coord: " << pt->innerVertex().transpose() << std::endl;
    cpt ++;
  };
  
  std::cout << "List of vertices: " << std::endl;
  auto it_pt = m_points.begin();
  auto pt = (*it_pt)->next();

  while(pt != *it_pt)
    {
      showPt(pt);
      pt = pt->next();
    }
  showPt(pt);
}

void StaticStabilityPolytope::computeHrep(Eigen::MatrixXd & Aineq, Eigen::VectorXd & bineq) const
{
  Aineq = Eigen::MatrixXd::Zero(m_points.size(),projectedPolytopeDim);
  bineq = Eigen::VectorXd::Zero(m_points.size());
  int counter = 0;
  for(auto pt : m_points)
  {
    Aineq.row(counter) = pt->plane().head(2).transpose(); //NOTE: use .head(2) because the third dimension is always zero!
    bineq(counter) = pt->plane()[3];
    counter++;
  }
}

std::vector<Eigen::Vector4d> StaticStabilityPolytope::constraintPlanes() const
{
  std::vector<Eigen::Vector4d> planes;  

  auto it_pt = m_points.begin();
  auto pt = (*it_pt)->next();
  
  // only insert the point if the normal is  not too close
  Eigen::Vector2d prevNormal = (*it_pt)->normal();
  double thres = 5e-2;
  while(pt != *it_pt)
    {
      auto normal = pt->normal();
      double dist = (normal-prevNormal).norm();
      if (dist >= thres)
	{
	  planes.push_back(pt->plane());
	  prevNormal = normal;
	}
      pt = pt->next();
    }
  planes.push_back(pt->plane());
  
  return planes;
}

std::vector<Eigen::Vector3d> StaticStabilityPolytope::vertices() const
{
  std::vector<Eigen::Vector3d> vertices;
  Eigen::Vector3d vertex;
  for (auto pt: m_points)
    {
      vertex << pt->innerVertex(), 0;
      vertices.push_back(vertex);
    }
  
  return vertices;
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

const void StaticStabilityPolytope::getRandomFeasiblePoint(Eigen::Vector2d & point) const
{
  //This method samples feasible points within the projected polytope in an efficient and fast manner.
  //However, the sampling does not enforce a uniform distribution!
  //
  //In future, one may choose a specific distribution of weights (i.e. uniform, normal, exponential, ...) 
  //as this affects the distribution of drawn points.
  point.setZero();
  
  Eigen::VectorXd weights = Eigen::VectorXd::Random(m_points.size()); //any distribution of weights results in a feasible point
  // The coefficients must be positive:
  weights = weights.cwiseAbs();
  // The sum of coefficient must be equal to 1.0 :
  const double sum = Eigen::VectorXd::Ones(m_points.size()).transpose()*weights; 
  weights /= sum;

  int counter = 0;
  for (auto pt: m_points) // the ordering of the vertices does not matter as the weights are random
  {
    point += weights(counter) * pt->innerVertex();
    counter ++;
  }
}

const bool StaticStabilityPolytope::getUniformRandomFeasiblePoint(Eigen::Vector2d & point, const int maxSamplingAttempts) const
{
  //This method samples feasible points within the projected polytope.
  //Drawn points are uniformly distributed.
  //The function's efficiency depends on the ratio between the volume of the enclosing hypercube and the volume of the projected polytope.
  //Therefore, the function may return false if `maxSamplingAttempts` is reached.

  //compute the hyper-cube that encloses the polytope
  Eigen::Vector2d maximums = (-1)*std::numeric_limits<double>::max()*Eigen::Vector2d::Ones();
  Eigen::Vector2d minimums = (+1)*std::numeric_limits<double>::max()*Eigen::Vector2d::Ones();
  for(auto pt: m_points)
  {
    for(int dim=0; dim<projectedPolytopeDim; dim++)
    {
      if(maximums[dim] < pt->innerVertex()[dim])
      {
        maximums[dim] = pt->innerVertex()[dim];
      }
      if(minimums[dim] > pt->innerVertex()[dim])
      {
        minimums[dim] = pt->innerVertex()[dim];
      }
    }
  }
  Eigen::Vector2d ranges = Eigen::Vector2d::Zero();
  for(int dim=0; dim<projectedPolytopeDim; dim++)
  {
    ranges[dim] = maximums[dim] - minimums[dim];
  }

  //uniformly sample points from the enclosing hypercube until a valid point is found
  bool valid = false;
  int counter = 0;
  while(!valid)
  {
    // std::cout << counter+1 << ". sampling attempt from hypercube" << std::endl;
    for(int dim=0; dim<projectedPolytopeDim; dim++)
    {
      double r = ((double) rand() / (RAND_MAX)); //generate a random number between 0 and 1
      point[dim] = minimums[dim] + r * ranges[dim];
    }
    valid = isPointFeasible(point);
    counter++;
    if(counter >= maxSamplingAttempts){
      point.setZero();
      return false;
    }
  }
  return true;
}

const bool StaticStabilityPolytope::isPointFeasible(Eigen::Vector2d & point) const
{
  // the point is feasible if it is in the half space of all the faces of the inner polytope.
  return std::all_of(m_points.begin(), m_points.end(), [&point](std::shared_ptr<StaticPoint> p){return p->pointInHalfSpace(point);});
}
