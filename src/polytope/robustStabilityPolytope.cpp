#include "polytope/robustStabilityPolytope.h"

// using namespace std;

// inherited constructor

RobustStabilityPolytope::~RobustStabilityPolytope()
{

  for(auto it : m_outerEdges)
  {
    it->finish();
  }

  for(auto it : m_faces)
  {
    it->finish();
  }


  // delete m_contactSetPtr;
  // m_contactSetPtr = nullptr;
}

// ----------- main class methods ----------
void RobustStabilityPolytope::initSolver()
{
  // m_contactSetPtr = static_cast<ContactSet*>(m_pdPtr);


  /*
    m_lp->buildProblem(m_contactSetPtr->buildVectorB(),
             m_contactSetPtr->buildMatrixA(),
             m_contactSetPtr->buildFrictionF(),
             m_contactSetPtr->buildFrictionVectorf());
  */
  /*
    m_lp->buildProblem(m_pdPtr->buildVectorB(), m_pdPtr->buildMatrixA(), m_pdPtr->buildFrictionF(),
                       m_pdPtr->buildFrictionVectorf());
           */
  m_pdPtr->update();
  m_lp->buildProblem(m_pdPtr->getVectorB(), m_pdPtr->getMatrixA(), m_pdPtr->getFrictionF(),
                     m_pdPtr->getFrictionVectorf());

  // Eigen::VectorXd b = m_pdPtr->getVectorB();
  // Eigen::MatrixXd A = m_pdPtr->getMatrixA();
  // Eigen::MatrixXd Ab(A.rows(), A.cols()+1);
  // Ab << A, b;
  // Eigen::MatrixXd F = m_pdPtr->getFrictionF();
  // Eigen::VectorXd f = m_pdPtr->getFrictionVectorf();
  // Eigen::MatrixXd Ff(F.rows(), F.cols()+1);
  // Ff << F, f;
  // std::cout << "Equalities: \n" << Ab << std::endl;
  // std::cout << "Friction: \n" << Ff << std::endl;
}

void RobustStabilityPolytope::solveLP(Eigen::Vector3d const & direction, Eigen::Vector3d & vertex)
{
  m_lp->set_searchDirection(direction);

  m_lp->solveProblem();

  vertex = m_lp->get_result();
}

void RobustStabilityPolytope::projectionStabilityPolyhedron()
{
  bool ok = this->computeProjectionStabilityPolyhedron();
  if(ok==false){
    std::cout << "\n method failed!" << std::endl; //TODO How to deal with this case?
  }
}

bool RobustStabilityPolytope::computeProjectionStabilityPolyhedron()
{
  // std::cout << "Reached here!" << '\n';
  std::vector<Eigen::Vector3d> initialDirections;
  std::vector<Eigen::Vector3d> initialPoints;
  Eigen::Vector3d dir;
  Eigen::Vector3d point;
  double theta, phi;
  
  std::shared_ptr<Vertex> newVertex;
  // std::shared_ptr<Face> dirFace;

  // Finding the first 4 points:
  //Taking random direction
  // double dist, area, volume;
  // Eigen::Vector3d v1, v2, v3;
  // srand(time(NULL));
  // while (initialPoints.size()<4)
  //   {
  //     // choose a random direction in R^3
  //     theta = 2*M_PI*(rand()%100)/100.0; // -> generate random number in [0, 2pi[
  //     phi = M_PI*(rand()%101)/100.0; // -> generates random number in [0, pi]
  //     std::cout << "Theta: " << theta << " phi: " << phi << std::endl;
      
  //     dir << sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi);
  //     dir = dir.normalized();

  //     // look for the point
  //     solveLP( dir, point);

  //     // check if the point is far enough of the other points
  //     bool validPoint = true;
  //     switch(initialPoints.size())
  // 	{
  // 	case 0:
  // 	  break;

  // 	case 1:
  // 	  v1 = point-initialPoints.at(0);
  // 	  dist = (v1).norm();
  // 	  if (dist < 0.1)
  // 	    {
  // 	      validPoint = false;
  // 	    }
  // 	  break;

  // 	case 2:
  // 	  v2 = point-initialPoints.at(0);
  // 	  area = (v1.cross(v2)).norm();
  // 	  if (area < 0.05)
  // 	    {
  // 	      validPoint = false;
  // 	    }
  // 	  break;

  // 	case 3:
  // 	  v3 = point-initialPoints.at(0);
  // 	  volume = abs((v1.cross(v2)).dot(v3));
  // 	  if  (volume < 0.01)
  // 	    {
  // 	      validPoint = false;
  // 	    }
  // 	  break;

  // 	default:
  // 	  throw(42);
  // 	  break;
  // 	}
      
  //     if (validPoint)
  // 	{
  // 	  initialPoints.push_back(point);
  // 	  initialDirections.push_back(dir);
  // 	}
  //     else
  // 	{
  // 	  std::cout << "Point rejected" << std::endl;
  // 	}
  //   }

  
  
  //   for (int i = 0; i< 4; i++)
  //   {
  //     auto pt = initialPoints.at(i);
  //     auto dir = initialDirections.at(i);

  //     newVertex = std::make_shared<Vertex>(pt, dir);
  //     m_vertices.push_back(newVertex);
  //   }

  dir << 0, 0, 1; 
  initialDirections.push_back(dir);
  for(int i = 1; i < 4; ++i)
  {
    dir << cos(2 * M_PI * (i - 1) / 3), sin(2 * M_PI * (i - 1) / 3), -1;
    dir = dir.normalized();
    initialDirections.push_back(dir);
  }

  for(auto d: initialDirections)
  {
    solveLP(d, point);
    for (auto pt: initialPoints)
    {
      if ((point-pt).norm() < 0.001) //TODO allow the user to modify this threshold
      {
        // std::cout << "Point is too close, stopping" << std::endl;
        return false;
      }
    }
    initialPoints.push_back(point);
    newVertex = std::make_shared<Vertex>(point, d);
    m_vertices.push_back(newVertex);
  }

  buildInnerPoly();
  buildOuterPoly();
  computeResidualFromScratch();

  bool ok;
  if(m_error<0){
    return false; //TODO
  }
  while(!stopCriterion())
  {
    m_iteration++;
    auto dirFace = *max_element(m_faces.begin(), m_faces.end(), Face::compareFacesMeasure);
    solveLP(dirFace->get_normal(), point);
    newVertex = std::make_shared<Vertex>(point, dirFace->get_normal());
    ok = updateInnerPoly(newVertex, dirFace);
    if(!ok)
    {
      return false;
    }
    if(m_faces.size()==0)
    {
      return false;
    }
    ok = updateOuterPoly(newVertex, dirFace);
    if(!ok)
    {
      return false;
    }
    updateSupportFunctions(dirFace);
    // à modifier -> à ne pas refaire de zéro
    computeResidualFromScratch();
  }
  return true;
}

Eigen::Vector3d RobustStabilityPolytope::computeInnerPoint()
{
  // Computation of the inner point: it is used to make sure that the normal of the faces are oriented toward the outside
  m_innerPoint = Eigen::Vector3d::Zero();
  for(auto it : m_vertices)
  {
    m_innerPoint += it->get_coordinates();
  }
  m_innerPoint /= m_vertices.size();

  return m_innerPoint;
}

void RobustStabilityPolytope::buildInnerPoly()
{
  computeInnerPoint();
  // std::cout << "Inner Point: " << m_innerPoint << '\n';

  std::shared_ptr<Edge> newEdge;
  std::shared_ptr<Face> newFace;

  newEdge = std::make_shared<Edge>(m_vertices[0], m_vertices[1]);
  m_edges.push_back(newEdge);
  newEdge = std::make_shared<Edge>(m_vertices[0], m_vertices[2]);
  m_edges.push_back(newEdge);
  newEdge = std::make_shared<Edge>(m_vertices[0], m_vertices[3]);
  m_edges.push_back(newEdge);
  newEdge = std::make_shared<Edge>(m_vertices[1], m_vertices[2]);
  m_edges.push_back(newEdge);
  newEdge = std::make_shared<Edge>(m_vertices[1], m_vertices[3]);
  m_edges.push_back(newEdge);
  newEdge = std::make_shared<Edge>(m_vertices[2], m_vertices[3]);
  m_edges.push_back(newEdge);

  newFace = std::make_shared<Face>(m_vertices[0], m_vertices[1], m_vertices[2], m_edges[0], m_edges[1], m_edges[3],
                                   m_innerPoint);
  m_faces.push_back(newFace);
  newFace = std::make_shared<Face>(m_vertices[0], m_vertices[1], m_vertices[3], m_edges[0], m_edges[2], m_edges[4],
                                   m_innerPoint);
  m_faces.push_back(newFace);
  newFace = std::make_shared<Face>(m_vertices[0], m_vertices[2], m_vertices[3], m_edges[1], m_edges[2], m_edges[5],
                                   m_innerPoint);
  m_faces.push_back(newFace);
  newFace = std::make_shared<Face>(m_vertices[1], m_vertices[2], m_vertices[3], m_edges[3], m_edges[4], m_edges[5],
                                   m_innerPoint);
  m_faces.push_back(newFace);
  newFace = nullptr;

  for(auto it : m_faces)
  {
    it->init();
  }
}

bool RobustStabilityPolytope::updateInnerPoly(std::shared_ptr<Vertex> & newVertex, std::shared_ptr<Face> & dirFace)
{
  std::list<std::shared_ptr<Face>> consideredFaces;
  std::vector<std::shared_ptr<Face>> currentNeighbors;

  // if the new point is in the inner approximation (that should not happen) or on the last face (this can happen), then there is 2 possibilities: i- reject the point but still update the outer approximation ii- split the search face in 3
  // For now I am rejecting the point but still updating the outer approximation
  if (dirFace->pointInHalfSpace(newVertex->get_coordinates()))
    {
      // std::cout << "Rejected point" << std::endl;
      // return;
      return true; //TODO or return false?
    }
    
  m_vertices.push_back(newVertex);
  
  consideredFaces.push_back(dirFace);
  
  std::vector<std::shared_ptr<Face>> visibleFaces;
  std::vector<std::shared_ptr<Edge>> visibleEdges;
  
  auto currentFace = consideredFaces.begin();

  // consider all the faces to be considered
  while(currentFace != consideredFaces.end())
  {
    // if the current face does not put the new point inside the inner polyhedron
    // then  it has to be removed
    if(!(*currentFace)->pointInHalfSpace(newVertex->get_coordinates()))
    {
      visibleFaces.push_back(*currentFace);

      // some edges sould be removed, for now it is done later after the
      // while loop
      auto currentEdges = (*currentFace)->get_edges();
      for(auto it_e : currentEdges)
      {
        if(find(visibleEdges.begin(), visibleEdges.end(), it_e) == visibleEdges.end())
        {
          visibleEdges.push_back(it_e);
        }
      }

      // the faces neighbor of the current visible face should be tested as
      // potential visible faces
      auto currentNeighbors = (*currentFace)->findNeighbors();
      for(auto it_f : currentNeighbors)
      {
        if(find(consideredFaces.begin(), consideredFaces.end(), it_f) == consideredFaces.end())
        {
          consideredFaces.push_back(it_f);
        }
      }
    }

    currentFace++;
  }

  

  
  // the discrimination of the edges that should be remove could be done in the previous loop
  std::vector<std::shared_ptr<Edge>> edges_to_keep;
  std::vector<std::shared_ptr<Edge>> edges_to_delete;

  // an edge is said to be visible if both the faces it belongs to are visible
  for(auto it : visibleEdges)
  {
    if(find(visibleFaces.begin(), visibleFaces.end(), it->get_face1()) != visibleFaces.end()
       and find(visibleFaces.begin(), visibleFaces.end(), it->get_face2()) != visibleFaces.end())
    {
      // std::cout << "Edge to delete: " << *it << '\n';
      edges_to_delete.push_back(it);
    }
    else
    {
      // std::cout << "Edge to keep: " << *it << '\n';
      edges_to_keep.push_back(it);
    }
  }

  
  
  // remove old faces
  for(auto it : visibleFaces)
  {
    auto posFace = find(m_faces.begin(), m_faces.end(), it);
    if(posFace != m_faces.end())
    {
      it->finish();
      *posFace = nullptr;
      m_faces.erase(posFace);
      // it = nullptr;
    }
    else
    {
      std::cout << "Visible face not found!" << '\n';
    }
  }

  // remove old edges:
  for(auto it : edges_to_delete)
  {
    auto posEdge = find(m_edges.begin(), m_edges.end(), it);
    if(posEdge != m_edges.end())
    {
      *posEdge = nullptr;
      m_edges.erase(posEdge);
    }
    else
    {
      std::cout << "Visible edge not found!" << '\n';
    }
  }

  
  // ---- create and add the new faces and edges of the inner polyhedron
  std::shared_ptr<Face> newFace;
  std::shared_ptr<Edge> newEdge1;
  std::shared_ptr<Edge> newEdge2;

  std::map<std::shared_ptr<Vertex>, std::shared_ptr<Edge>> processedVertex;

  for(auto it : edges_to_keep)
  {
    if(processedVertex.find(it->get_vertex1()) == processedVertex.end())
    {
      newEdge1 = std::make_shared<Edge>(newVertex, it->get_vertex1());
      m_edges.push_back(newEdge1);
      processedVertex[it->get_vertex1()] = newEdge1;
    }
    else
    {
      newEdge1 = processedVertex.find(it->get_vertex1())->second;
    }

    if(processedVertex.find(it->get_vertex2()) == processedVertex.end())
    {
      newEdge2 = std::make_shared<Edge>(newVertex, it->get_vertex2());
      m_edges.push_back(newEdge2);
      processedVertex[it->get_vertex2()] = newEdge2;
    }
    else
    {
      newEdge2 = processedVertex.find(it->get_vertex2())->second;
    }

    // std::cout << "Old edge: "<< it->get_index()<<", new edge 1: " << newEdge1->get_index() << ", new edge 2: "<<
    // newEdge2->get_index() << '\n'; showPoly();
    newFace =
        std::make_shared<Face>(newVertex, it->get_vertex1(), it->get_vertex2(), it, newEdge1, newEdge2, m_innerPoint);
    bool ok = newFace->init();
    if(!ok){
      return false;
    }
    m_faces.push_back(newFace);
  }
  return true;
}

void RobustStabilityPolytope::buildOuterPoly()
{
  std::shared_ptr<OuterFace> newOuterFace;

  for(auto & it : m_vertices)
  {
    newOuterFace = std::make_shared<OuterFace>(it);
    m_outerFaces.push_back(newOuterFace);
    m_innerOuterLink[it] = newOuterFace;
  }
  newOuterFace = nullptr;

  std::shared_ptr<OuterVertex> newOuterVertex;
  for(auto & it : m_faces)
  {
    // std::cout << "Face " << it->get_index() << '\n';
    newOuterVertex =
        std::make_shared<OuterVertex>(m_innerOuterLink.at(it->get_vertex1()), m_innerOuterLink.at(it->get_vertex2()),
                                      m_innerOuterLink.at(it->get_vertex3()));
    m_outerVertices.push_back(newOuterVertex);
    it->set_supportPoint(newOuterVertex);
  }
  newOuterVertex = nullptr;

  std::shared_ptr<OuterEdge> newOuterEdge;

  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(0), m_outerVertices.at(1));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);
  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(0), m_outerVertices.at(2));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);
  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(0), m_outerVertices.at(3));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);
  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(1), m_outerVertices.at(2));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);
  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(1), m_outerVertices.at(3));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);
  newOuterEdge = std::make_shared<OuterEdge>(m_outerVertices.at(2), m_outerVertices.at(3));
  newOuterEdge->init();
  m_outerEdges.push_back(newOuterEdge);

  newOuterEdge = nullptr;
}

bool RobustStabilityPolytope::updateOuterPoly(std::shared_ptr<Vertex> & newVertex, std::shared_ptr<Face> & dirFace)
{
  // --------- Double Description Method ---------
  std::vector<std::shared_ptr<OuterVertex>> U_0; // outer vertex on the plane
  std::vector<std::shared_ptr<OuterVertex>> U_minus; // outer vertices that are outside the new plane

  std::vector<std::shared_ptr<OuterEdge>> E_0; // edges that are intersected
  std::vector<std::shared_ptr<OuterEdge>> E_new; // new edges created on the new plane
  std::vector<std::shared_ptr<OuterEdge>> E_minus; // edges that are fully outside

  std::vector<std::shared_ptr<OuterFace>> F_0; // faces that are intersecting with the plane
  // std::vector<std::shared_ptr<OuterFace>> F_minus; // faces that are fully outside: there should be none
  std::shared_ptr<OuterFace> F_new = std::make_shared<OuterFace>(newVertex); // new face there should be only one
  m_innerOuterLink[newVertex] = F_new;
  m_outerFaces.push_back(F_new);

  std::list<std::shared_ptr<OuterVertex>> consideredPoints;
  std::vector<std::shared_ptr<OuterVertex>> currentNeighbors;
  std::list<std::shared_ptr<OuterEdge>> consideredEdges;
  // std::vector<std::shared_ptr<OuterEdge>> currentEdges;

  consideredPoints.push_back(dirFace->get_supportPoint());

  std::shared_ptr<OuterVertex> newOuterVertex;
  double d1(0), d2(0);
  Eigen::Vector3d coord;


  // std::cout << "dot 1" << '\n';
  auto currentPoint = consideredPoints.begin();

  // find the vertex of the previous outer approximation that are cut out by
  // the new plane. Then cut the adjacent edages and faces.
  // The vertex that are now out are found using a neigbor search:
  // To be proven (or not) there is a path of edges that goes from any vertex
  // that is cut out to any other vertex that is cut out
  while(currentPoint != consideredPoints.end())
  {
    // std::cout << "Current point: " << *currentPoint << '\n';
    // std::cout << "dot 2" << '\n';
    if(!(*currentPoint)->strictlyContainedInHalfspace(F_new))
    {
      auto currentEdges = (*currentPoint)->get_outerEdges();
      // std::cout << "Number of edges: " << currentEdges.size() << '\n';
      for(auto it_e : currentEdges) // all points only have 3 edges because of the way the outer polyhedron is built
      {
        if(find(consideredEdges.begin(), consideredEdges.end(), it_e) == consideredEdges.end())
        {
          consideredEdges.push_back(it_e);
          auto otherOuterVertex = it_e->get_otherOuterVertex(*currentPoint);

          if(!(otherOuterVertex->strictlyContainedInHalfspace(F_new)))
          {
            // std::cout << "Reached here! (yes)" << '\n';
            // E_minus.push_back(it);
            if(find(consideredPoints.begin(), consideredPoints.end(), otherOuterVertex) == consideredPoints.end())
            {
              // std::cout << "Adding another vertex to consider: " << otherOuterVertex << '\n';
              consideredPoints.push_back(otherOuterVertex);
            }

            auto posEdge = find(m_outerEdges.begin(), m_outerEdges.end(), it_e);
            if(posEdge != m_outerEdges.end())
            {
              it_e->finish();
              *posEdge = nullptr;
              m_outerEdges.erase(posEdge);
            }
            else
            {
              return false;
              std::cerr << "Outer Edge as already been removed!" << '\n';
            }
          }
          else
          {
            E_0.push_back(it_e);
            // cut the edges by creating new points
            d1 = abs(F_new->get_normal().dot(it_e->get_outerVertex1()->get_coordinates()) - F_new->get_offset());
            d2 = abs(F_new->get_normal().dot(it_e->get_outerVertex2()->get_coordinates()) - F_new->get_offset());
            coord = d1 * it_e->get_outerVertex2()->get_coordinates() + d2 * it_e->get_outerVertex1()->get_coordinates();
            coord /= d1 + d2;

            newOuterVertex = std::make_shared<OuterVertex>(coord);

            newOuterVertex->add_outerFace(F_new);
            newOuterVertex->add_outerFace(it_e->get_outerFace1());
            newOuterVertex->add_outerFace(it_e->get_outerFace2());

            it_e->switch_outerVertices(*currentPoint, newOuterVertex);

            m_outerVertices.push_back(newOuterVertex);
            U_0.push_back(newOuterVertex);

            // the edge's faces don't change, add then to F_0
            // std::cout << "Came here but nothing append!" << '\n';
            it_e->add_outerFacesToVector(F_0);
            // std::cout << "But not here! (no)" << '\n';
          }
        }
      }
      // U_minus.push_back(*currentPoint);
      auto posVert = find(m_outerVertices.begin(), m_outerVertices.end(), *currentPoint);
      if(posVert != m_outerVertices.end())
      {
        *posVert = nullptr;
        m_outerVertices.erase(posVert);
      }
      else
      {
        return false;
        std::cerr << "Outer Vertex has already been removed!" << '\n';
      }
    }
    // std::cout << "Nani 1!" << '\n';

    currentPoint++;
    // std::cout << "Nani 1!" << '\n';
  }

  // std::cout << "Number of considered points: " << consideredPoints.size() << " with " << U_minus.size() << " points

  // ---- creating the new stuff: 1 new face, some new edges and as many new vertex.
  // by construction 1 outer face may have any number of edges but each outer
  // vertex has only 3 edges that reach it.

  std::shared_ptr<OuterEdge> newOuterEdge;


  for(auto it : F_0)
  {
    std::shared_ptr<OuterVertex> outerVertex1, outerVertex2;

    auto it_vert = U_0.begin();
    // std::vector<std::shared_ptr<OuterFace>> vertexOuterFaces;

    bool v1Found(false), v2Found(false);

    while(it_vert != U_0.end() and !v1Found)
    {
      auto vertexOuterFaces = (*it_vert)->get_outerFaces();
      if(find(vertexOuterFaces.begin(), vertexOuterFaces.end(), it) != vertexOuterFaces.end())
      {
        v1Found = true;
        outerVertex1 = *it_vert;
      }
      it_vert++;
    }

    while(it_vert != U_0.end() and !v2Found)
    {
      auto vertexOuterFaces = (*it_vert)->get_outerFaces();
      if(find(vertexOuterFaces.begin(), vertexOuterFaces.end(), it) != vertexOuterFaces.end())
      {
        v2Found = true;
        outerVertex2 = *it_vert;
      }
      it_vert++;
    }

    // std::cout << "Vertex 1: " << outerVertex1 << ", vertex 2: " << outerVertex2 << '\n';

    newOuterEdge = std::make_shared<OuterEdge>(outerVertex1, outerVertex2);
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
  }
  return true;
}

void RobustStabilityPolytope::updateSupportFunctions(std::shared_ptr<Face> & dirFace)
{

  std::list<std::shared_ptr<Face>> consideredFaces;

  // need to init with a first few considered faces
  if(!dirFace.unique())
  {
    consideredFaces.push_back(dirFace);
  }

    // std::cout << "############ DEBUG m_faces.size() = " << m_faces.size() << std::endl;
  consideredFaces.push_back(m_faces.back());

  auto currentFace = consideredFaces.begin();

  while(currentFace != consideredFaces.end())
  {
    // m_innerOuterLink[it_face->get_vertex1()]; -> gives an outer face but don't give a starting point
    auto initPoint = m_outerVertices.at(0);
    // std::cout << "Current Support Function: " << (*currentFace)->get_supportFunction() <<'\n';

    if(computeSupportFunction(*currentFace, initPoint))
    {
      // add neighbors
      auto currentNeighbors = (*currentFace)->findNeighbors();
      for(auto it : currentNeighbors)
      {
        if(find(consideredFaces.begin(), consideredFaces.end(), it) == consideredFaces.end())
        {
          // std::cout << it->get_index() << '\n';
          consideredFaces.push_back(it);
        }
      }
    }
    // std::cout << "New Support Function: " << (*currentFace)->get_supportFunction() <<'\n';

    currentFace++;
  }
}

bool RobustStabilityPolytope::computeSupportFunction(std::shared_ptr<Face> & face,
                                                     const std::shared_ptr<OuterVertex> & initPoint)
{
  // std::cout << "Computing support function for face " << face->get_index() << '\n';
  double prevSupportFunction = face->get_supportFunction();
  auto currentOuterVertex = initPoint;
  double currentDistance = face->get_normal().dot(currentOuterVertex->get_coordinates()) - face->get_offset();
  double distance = 100;

  std::vector<std::shared_ptr<OuterVertex>> visitedPoints;
  visitedPoints.push_back(currentOuterVertex);

  bool currentIsSupport = false;

  while(!currentIsSupport)
  {
    currentIsSupport = true;
    auto currentNeighbors = currentOuterVertex->findNeighbors();

    for(auto it_v : currentNeighbors)
    {
      if(find(visitedPoints.begin(), visitedPoints.end(), it_v) == visitedPoints.end())
      {
        visitedPoints.push_back(it_v);
        distance = face->get_normal().dot(it_v->get_coordinates()) - face->get_offset();
        if(distance > currentDistance)
        {
          currentDistance = distance;
          currentOuterVertex = it_v;
          currentIsSupport = false;
        }
      }
    }
  }
  face->set_supportPoint(currentOuterVertex);

  if(abs(prevSupportFunction - face->get_supportFunction()) < 0.00000001)
  {
    return false;
  }
  else
  {
    // m_error += face->get_area()*(face->get_supportFunction()-prevSupportFunction);
    return true;
  }
}

double RobustStabilityPolytope::computeResidualFromScratch()
{
  m_error = 0;
  for(auto it : m_faces)
  {
    m_error += it->get_area() * it->get_supportFunction();
  }
  return m_error;
}

// ----------- output and display functions ----------
void RobustStabilityPolytope::writeToStream(std::ofstream & stream) const
{

  if(stream)
  {

    for(auto it_vertices : m_vertices)
    {
      stream << "iv;" // iv = inner vertices
             << it_vertices->get_coordinates()[0] << ';' << it_vertices->get_coordinates()[1] << ';'
             << it_vertices->get_coordinates()[2] << ';' << it_vertices->get_direction()[0] << ';'
             << it_vertices->get_direction()[1] << ';' << it_vertices->get_direction()[2] << ';' << std::endl;
    }

    for(auto it : m_edges)
    {
      stream << "ie;" // ie = inner edge
             << it->get_vertex1()->get_coordinates()[0] << ';' << it->get_vertex1()->get_coordinates()[1] << ';'
             << it->get_vertex1()->get_coordinates()[2] << ';' << it->get_vertex2()->get_coordinates()[0] << ';'
             << it->get_vertex2()->get_coordinates()[1] << ';' << it->get_vertex2()->get_coordinates()[2] << ';'
             << std::endl;
    }

    for(auto const & it : m_outerVertices)
    {
      stream << "ov;" // oe = outer vertex
             << it->get_coordinates()[0] << ';' << it->get_coordinates()[1] << ';' << it->get_coordinates()[2] << ';'
             << std::endl;
    }

    for(auto const & it : m_outerEdges)
    {
      stream << "oe;" // oe = outer edge
             << it->get_outerVertex1()->get_coordinates()[0] << ';' << it->get_outerVertex1()->get_coordinates()[1]
             << ';' << it->get_outerVertex1()->get_coordinates()[2] << ';'
             << it->get_outerVertex2()->get_coordinates()[0] << ';' << it->get_outerVertex2()->get_coordinates()[1]
             << ';' << it->get_outerVertex2()->get_coordinates()[2] << ';' << std::endl;
    }
  }
  else
  {
    std::cerr << "Error: Output Stream not open." << '\n';
  }
}

void RobustStabilityPolytope::showPoly() const
{
  std::cout << "Current Vertices: ";
  for(auto it : m_vertices)
  {
    std::cout << it->get_index() << ", ";
  }
  std::cout << '\n';

  std::cout << "Current Edges: " << '\n';
  for(auto it : m_edges)
  {
    std::cout << "Edge: " << it->get_index() << " has vertex " << it->get_vertex1()->get_index() << " and "
              << it->get_vertex2()->get_index() << '\n';
  }

  std::cout << '\n';
}

void RobustStabilityPolytope::computeHrep(Eigen::MatrixXd & Aineq, Eigen::VectorXd & bineq) const
{
  Aineq = Eigen::MatrixXd::Zero(m_faces.size(),projectedPolytopeDim);
  bineq = Eigen::VectorXd::Zero(m_faces.size());
  int counter = 0;
  for(auto face : m_faces)
  {
    Aineq.row(counter) = face->get_normal().transpose();
    bineq(counter) = face->get_offset();
    counter++;
  }
}

std::vector<Eigen::Vector4d> RobustStabilityPolytope::constraintPlanes() const
{
  std::vector<Eigen::Vector4d> planes;

  for(auto face : m_faces)
  {
    Eigen::Vector4d plane;
    plane << face->get_normal(), face->get_offset();

    planes.push_back(plane);
  }

  return planes;
}

Eigen::Vector3d RobustStabilityPolytope::baryPoint() const
{
  Eigen::Vector3d baryPt = Eigen::Vector3d::Zero();

  for(auto pt : m_vertices)
  {
    baryPt += pt->get_coordinates();
  }

  baryPt /= m_vertices.size();

  return baryPt;
}

// ----------- getters ----------
int RobustStabilityPolytope::get_numberOfVertices() const
{
  return m_vertices.size();
}

int RobustStabilityPolytope::get_numberOfFaces() const
{
  return m_faces.size();
}

int RobustStabilityPolytope::get_numberOfOuterVertices() const
{
  return m_outerVertices.size();
}

int RobustStabilityPolytope::get_numberOfOuterFaces() const
{
  return m_outerFaces.size();
}

double RobustStabilityPolytope::get_innerConvexMicro() const
{
  return m_innerConvexMicro;
}

double RobustStabilityPolytope::get_outerConvexMicro() const
{
  return m_outerConvexMicro;
}

double RobustStabilityPolytope::get_supportFunctionMicro() const
{
  return m_supportFunctionMicro;
}

std::vector<Eigen::Vector3d> RobustStabilityPolytope::get_innerFaceNormals() const
{
  std::vector<Eigen::Vector3d> innerFaceNormals;
  for(auto face : m_faces)
  {
    innerFaceNormals.push_back(face->get_normal());
  }
  return innerFaceNormals;
}

std::vector<double> RobustStabilityPolytope::get_innerFaceOffsets() const
{
  std::vector<double> innerFaceOffsets;
  for(auto face : m_faces)
  {
    innerFaceOffsets.push_back(face->get_offset());
  }
  return innerFaceOffsets;
}

const std::vector<Eigen::Vector3d> RobustStabilityPolytope::getInnerVertices() const
{
  std::vector<Eigen::Vector3d> vertices;
  for(auto it : m_vertices)
  {
    // std::cout << "cor " << it->get_coordinates().transpose() << std::endl;
    // std::cout << "dir " << it->get_direction().transpose() << std::endl;
    vertices.push_back( it->get_coordinates() );
  }
  return vertices;
}

const void RobustStabilityPolytope::getRandomFeasiblePoint(Eigen::Vector3d & point) const
{
  point.setZero();

  Eigen::VectorXd weights = Eigen::VectorXd::Random(m_vertices.size());
  // The coefficients must be positive:
  weights = weights.cwiseAbs();
  // The sum of coefficient must be equal to 1.0 :
  const double sum = Eigen::VectorXd::Ones(m_vertices.size()).transpose()*weights; 
  weights /= sum;

  int counter = 0;
  for(auto it : m_vertices)
  {
    point += weights(counter) * it->get_coordinates();
    counter++;
  }
}

const bool RobustStabilityPolytope::getUniformRandomFeasiblePoint(Eigen::Vector3d & point) const
{
  //compute the hyper-cube that encloses the polytope
  Eigen::Vector3d maximums = (-1)*std::numeric_limits<double>::max()*Eigen::Vector3d::Ones();
  Eigen::Vector3d minimums = (+1)*std::numeric_limits<double>::max()*Eigen::Vector3d::Ones();
  for(auto it : m_vertices)
  {
    for(int dim=0; dim<projectedPolytopeDim; dim++)
    {
      if(maximums[dim] < it->get_coordinates()[dim])
      {
        maximums[dim] = it->get_coordinates()[dim];
      }
      if(minimums[dim] > it->get_coordinates()[dim])
      {
        minimums[dim] = it->get_coordinates()[dim];
      }
    }
  }
  double volumeCube = 0;
  Eigen::Vector3d ranges = Eigen::Vector3d::Zero();
  for(int dim=0; dim<projectedPolytopeDim; dim++)
  {
    ranges[dim] = maximums[dim] - minimums[dim];
    if(dim==0){
      volumeCube = ranges[dim];
    }
    else{
      volumeCube *= ranges[dim];
    }
  }
  const double volumePolytope = volumeCube; //TODO: how to obtain the projected polytope volume?
  const double ratio = volumePolytope / volumeCube;
  // std::cout << "ratio " << ratio << std::endl;
  if (ratio < 0.01){
    return false; //if the ratio is very small, then this uniform-sampling approach will take a long time...
  }

  //uniformly sample points from the enclosing hypercube until a valid point is found
  bool valid = false;
  int counter = 0;
  const int maxSamplingAttempts = 100;
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
      return false;
    }
  }
  return true;
}

const bool RobustStabilityPolytope::isPointFeasible(Eigen::Vector3d & point) const
{
  Eigen::MatrixXd Aineq;
  Eigen::VectorXd bineq;
  this->computeHrep(Aineq, bineq);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(bineq.size());
  res = Aineq * point - bineq;
  if(res.maxCoeff() > 0.0){
    return false;
  }
  else{
    return true;
  }
}



// ------------------ setter -----------------------
