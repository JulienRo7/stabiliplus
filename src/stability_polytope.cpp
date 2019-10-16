#include "stabiliplus/stability_polytope.h"


// using namespace std;

StabilityPolytope::StabilityPolytope(ContactSet contactSet, int maxNumberOfIteration, Solver solver):
m_contactSet(contactSet),
m_numberOfIterations(0), m_maxNumberOfIteration(maxNumberOfIteration), m_residual(1000),
m_lpMicro(0), m_innerConvexMicro(0), m_outerConvexMicro(0), m_supportFunctionMicro(0),
m_solver(solver)
{
  switch(m_solver)
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

  for (auto it: m_outerEdges)
  {
    it->finish();
  }

  for (auto it: m_faces)
  {
    it->finish();
  }
}


// ----------- main class methods ----------
void StabilityPolytope::buildStabilityProblem()
{
  m_lp->buildProblem(m_contactSet.buildVectorB(),
  		     m_contactSet.buildMatrixA(),
  		     m_contactSet.buildFrictionF(),
  		     m_contactSet.buildFrictionVectorf());

  // Eigen::VectorXd B = m_contactSet.buildVectorB();
  // Eigen::MatrixXd A = m_contactSet.buildMatrixA();
  // std::cout << "A: "<< '\n' << A << '\n';
  // Eigen::MatrixXd F = m_contactSet.buildFrictionF();
  // std::cout << "F: "<< '\n' << F << '\n';
  // Eigen::VectorXd f = m_contactSet.buildFrictionVectorf();


}


void StabilityPolytope::solveStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point)
{

  m_lp->set_searchDirection(direction);

  m_lp->solveProblem();
  
  point = m_lp->get_result();
}

void StabilityPolytope::projectionStabilityPolyhedron()
{
    // std::cout << "Reached here!" << '\n';
    std::array<Eigen::Vector3d, 4> initialDirections;
    Eigen::Vector3d dir;
    Eigen::Vector3d point;

    std::shared_ptr<Vertex> newVertex;
    // std::shared_ptr<Face> dirFace;

    dir << 0,0,1;
    initialDirections[0] = dir;
    for (int i=1; i<4; ++i)
    {
        dir << cos(2*M_PI*(i-1)/3), sin(2*M_PI*(i-1)/3), -1;
        dir = dir.normalized();
        initialDirections[i] = dir;
    }

    for (int i=0; i<4; ++i)
    {
        auto start = std::chrono::high_resolution_clock::now();
        solveStabilityProblem(initialDirections[i], point);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_lpMicro += duration.count();

        newVertex = std::make_shared<Vertex> (point, initialDirections[i]);
        m_vertices.push_back(newVertex);
    }

    auto start = std::chrono::high_resolution_clock::now();
    buildInnerPoly();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    m_innerConvexMicro+=duration.count();

    start = std::chrono::high_resolution_clock::now();
    buildOuterPoly();
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    m_outerConvexMicro+=duration.count();

    computeResidualFromScratch();

    while (stopCriterion())
    {

        m_numberOfIterations++;
        // std::cout << "Iteration number: " << m_numberOfIterations << ", residual: " << m_residual << '\n';
        auto dirFace = *max_element(m_faces.begin(), m_faces.end(), Face::compareFacesMeasure);


        auto start = std::chrono::high_resolution_clock::now();
        solveStabilityProblem(dirFace->get_normal(), point);
        // solveStabilityProblem(dirFace->get_normal(), point);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_lpMicro += duration.count();

        // const double check = dirFace->get_normal().dot(point) - dirFace->get_offset();

        newVertex = std::make_shared<Vertex> (point, dirFace->get_normal());
        m_vertices.push_back(newVertex);


        // std::cout << "Next research face: " << dirFace->get_index() << " with measure: " << dirFace->get_measure() <<'\n';
        start = std::chrono::high_resolution_clock::now();
        updateInnerPoly(newVertex, dirFace);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_innerConvexMicro+=duration.count();

        start = std::chrono::high_resolution_clock::now();
        updateOuterPoly(newVertex, dirFace);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_outerConvexMicro+=duration.count();

        start = std::chrono::high_resolution_clock::now();
        updateSupportFunctions(dirFace);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_supportFunctionMicro+=duration.count();

        // à modifier -> à ne pas refaire de zéro
        computeResidualFromScratch();
    }
}


Eigen::Vector3d StabilityPolytope::computeInnerPoint()
{
    // Computation of the inner point: it is used to make sure that the normal of the faces are oriented toward the outside
    m_innerPoint = Eigen::Vector3d::Zero();
    for (auto it : m_vertices)
    {
        m_innerPoint += it->get_coordinates();
    }
    m_innerPoint /= m_vertices.size();
}

void StabilityPolytope::buildInnerPoly()
{
    computeInnerPoint();
    // std::cout << "Inner Point: " << m_innerPoint << '\n';

    std::shared_ptr<Edge> newEdge;
    std::shared_ptr<Face> newFace;

    newEdge = std::make_shared<Edge> (m_vertices[0], m_vertices[1]);
    m_edges.push_back(newEdge);
    newEdge = std::make_shared<Edge> (m_vertices[0], m_vertices[2]);
    m_edges.push_back(newEdge);
    newEdge = std::make_shared<Edge> (m_vertices[0], m_vertices[3]);
    m_edges.push_back(newEdge);
    newEdge = std::make_shared<Edge> (m_vertices[1], m_vertices[2]);
    m_edges.push_back(newEdge);
    newEdge = std::make_shared<Edge> (m_vertices[1], m_vertices[3]);
    m_edges.push_back(newEdge);
    newEdge = std::make_shared<Edge> (m_vertices[2], m_vertices[3]);
    m_edges.push_back(newEdge);

    newFace = std::make_shared<Face> (m_vertices[0], m_vertices[1], m_vertices[2], m_edges[0], m_edges[1], m_edges[3], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = std::make_shared<Face> (m_vertices[0], m_vertices[1], m_vertices[3], m_edges[0], m_edges[2], m_edges[4], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = std::make_shared<Face> (m_vertices[0], m_vertices[2], m_vertices[3], m_edges[1], m_edges[2], m_edges[5], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = std::make_shared<Face> (m_vertices[1], m_vertices[2], m_vertices[3], m_edges[3], m_edges[4], m_edges[5], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = nullptr;


    for (auto it : m_faces)
    {
        it->init();
    }

}

void StabilityPolytope::updateInnerPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace)
{
    std::list<std::shared_ptr<Face>> consideredFaces;
    std::vector<std::shared_ptr<Face>> currentNeighbors;

    consideredFaces.push_back(dirFace);

    std::vector<std::shared_ptr<Face>> visibleFaces;
    std::vector<std::shared_ptr<Edge>> visibleEdges;

    auto currentFace = consideredFaces.begin();

    // consider all the faces to be considered
    while (currentFace!=consideredFaces.end())
    {
        // if the current face does not put the new point inside the inner polyhedron
        // then  it has to be removed
        if (!(*currentFace)->pointInHalfSpace(newVertex->get_coordinates()))
        {
            visibleFaces.push_back(*currentFace);

            // some edges sould be removed, for now it is done later after the
            // while loop
            auto currentEdges = (*currentFace)->get_edges();
            for (auto it_e : currentEdges)
            {
                if (find(visibleEdges.begin(), visibleEdges.end(), it_e)==visibleEdges.end())
                {
                    visibleEdges.push_back(it_e);
                }
            }

            // the faces neighbor of the current visible face should be tested as
            // potential visible faces
            auto currentNeighbors = (*currentFace)->findNeighbors();
            for (auto it_f : currentNeighbors)
            {
                if (find(consideredFaces.begin(), consideredFaces.end(), it_f)==consideredFaces.end())
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
    for (auto it : visibleEdges)
    {
        if (find(visibleFaces.begin(), visibleFaces.end(), it->get_face1())!=visibleFaces.end() and
            find(visibleFaces.begin(), visibleFaces.end(), it->get_face2())!=visibleFaces.end())
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


    //remove old faces
    for (auto it : visibleFaces)
    {
        auto posFace = find(m_faces.begin(), m_faces.end(), it);
        if (posFace!= m_faces.end())
        {
            it->finish();
            *posFace=nullptr;
            m_faces.erase(posFace);
            // it = nullptr;
        }
        else
        {
            std::cout << "Visible face not found!" << '\n';
        }
    }


    // remove old edges:
    for (auto it : edges_to_delete)
    {
        auto posEdge = find(m_edges.begin(), m_edges.end(), it);
        if (posEdge!= m_edges.end())
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

    for ( auto it : edges_to_keep)
    {
        if (processedVertex.find(it->get_vertex1())==processedVertex.end())
        {
            newEdge1 = std::make_shared<Edge> (newVertex, it->get_vertex1());
            m_edges.push_back(newEdge1);
            processedVertex[it->get_vertex1()] = newEdge1;
        }
        else
        {
            newEdge1 = processedVertex.find(it->get_vertex1())->second;
        }

        if (processedVertex.find(it->get_vertex2())==processedVertex.end())
        {
            newEdge2 = std::make_shared<Edge> (newVertex, it->get_vertex2());
            m_edges.push_back(newEdge2);
            processedVertex[it->get_vertex2()] = newEdge2;
        }
        else
        {
            newEdge2 = processedVertex.find(it->get_vertex2())->second;
        }

        // std::cout << "Old edge: "<< it->get_index()<<", new edge 1: " << newEdge1->get_index() << ", new edge 2: "<< newEdge2->get_index() << '\n';
        // showPoly();
        newFace = std::make_shared<Face> (newVertex, it->get_vertex1(), it->get_vertex2(), it, newEdge1, newEdge2, m_innerPoint);
        newFace->init();
        m_faces.push_back(newFace);

    }
}

void StabilityPolytope::buildOuterPoly()
{
    std::shared_ptr<OuterFace> newOuterFace;

    for (auto& it : m_vertices)
    {
        newOuterFace = std::make_shared<OuterFace> (it);
        m_outerFaces.push_back(newOuterFace);
        m_innerOuterLink[it] = newOuterFace;
    }
    newOuterFace = nullptr;

    std::shared_ptr<OuterVertex> newOuterVertex;
    for (auto& it : m_faces)
    {
        // std::cout << "Face " << it->get_index() << '\n';
        newOuterVertex = std::make_shared<OuterVertex>(m_innerOuterLink.at(it->get_vertex1()), m_innerOuterLink.at(it->get_vertex2()), m_innerOuterLink.at(it->get_vertex3()));
        m_outerVertices.push_back(newOuterVertex);
        it->set_supportPoint(newOuterVertex);
    }
    newOuterVertex = nullptr;

    std::shared_ptr<OuterEdge> newOuterEdge;

    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(0), m_outerVertices.at(1));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(0), m_outerVertices.at(2));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(0), m_outerVertices.at(3));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(1), m_outerVertices.at(2));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(1), m_outerVertices.at(3));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);
    newOuterEdge = std::make_shared<OuterEdge> (m_outerVertices.at(2), m_outerVertices.at(3));
    newOuterEdge->init();
    m_outerEdges.push_back(newOuterEdge);

    newOuterEdge = nullptr;

}

void StabilityPolytope::updateOuterPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace)
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

    // auto start = std::chrono::high_resolution_clock::now();
    // std::cout << "dot 1" << '\n';
    auto currentPoint = consideredPoints.begin();

    // find the vertex of the previous outer approximation that are cut out by
    // the new plane. Then cut the adjacent edages and faces.
    // The vertex that are now out are found using a neigbor search:
    // To be proven (or not) there is a path of edges that goes from any vertex
    // that is cut out to any other vertex that is cut out
    while (currentPoint != consideredPoints.end())
    {
        // std::cout << "Current point: " << *currentPoint << '\n';
        // std::cout << "dot 2" << '\n';
        if (!(*currentPoint)->strictlyContainedInHalfspace(F_new))
        {
            auto currentEdges = (*currentPoint)->get_outerEdges();
            // std::cout << "Number of edges: " << currentEdges.size() << '\n';
            for (auto it_e: currentEdges) // all points only have 3 edges because of the way the outer polyhedron is built
            {
                if (find(consideredEdges.begin(), consideredEdges.end(), it_e)==consideredEdges.end())
                {
                    consideredEdges.push_back(it_e);
                    auto otherOuterVertex = it_e->get_otherOuterVertex(*currentPoint);

                    if (!(otherOuterVertex->strictlyContainedInHalfspace(F_new)))
                    {
                        // std::cout << "Reached here! (yes)" << '\n';
                        // E_minus.push_back(it);
                        if (find(consideredPoints.begin(), consideredPoints.end(), otherOuterVertex)==consideredPoints.end())
                        {
                            // std::cout << "Adding another vertex to consider: " << otherOuterVertex << '\n';
                            consideredPoints.push_back(otherOuterVertex);
                        }

                        auto posEdge = find(m_outerEdges.begin(), m_outerEdges.end(), it_e);
                        if (posEdge!=m_outerEdges.end())
                        {
                            it_e->finish();
                            *posEdge = nullptr;
                            m_outerEdges.erase(posEdge);
                        }
                        else
                        {
                            std::cerr << "Outer Edge as already been removed!" << '\n';
                        }
                    }
                    else
                    {
                        E_0.push_back(it_e);
                        // cut the edges by creating new points
                        d1 = abs(F_new->get_normal().dot(it_e->get_outerVertex1()->get_coordinates())-F_new->get_offset());
                        d2 = abs(F_new->get_normal().dot(it_e->get_outerVertex2()->get_coordinates())-F_new->get_offset());
                        coord = d1*it_e->get_outerVertex2()->get_coordinates() + d2*it_e->get_outerVertex1()->get_coordinates();
                        coord /= d1+d2;

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
            if (posVert!=m_outerVertices.end())
            {
                *posVert=nullptr;
                m_outerVertices.erase(posVert);
            }
            else
            {
                std::cerr << "Outer Vertex has already been removed!" << '\n';
            }
        }
        // std::cout << "Nani 1!" << '\n';

        currentPoint++;
        // std::cout << "Nani 1!" << '\n';
    }
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // std::cout << "Number of considered points: " << consideredPoints.size() << " with " << U_minus.size() << " points out" << '\n';
    // std::cout << "First part duration: " << duration.count() << " microseconds" << '\n';

    // ---- creating the new stuff: 1 new face, some new edges and as many new vertex.
    // by construction 1 outer face may have any number of edges but each outer
    // vertex has only 3 edges that reach it.

    std::shared_ptr<OuterEdge> newOuterEdge;

    // start = std::chrono::high_resolution_clock::now()
    for (auto it : F_0)
    {
        std::shared_ptr<OuterVertex> outerVertex1, outerVertex2;

        auto it_vert = U_0.begin();
        // std::vector<std::shared_ptr<OuterFace>> vertexOuterFaces;

        bool v1Found(false), v2Found(false);

        while (it_vert!=U_0.end() and !v1Found)
        {
            auto vertexOuterFaces = (*it_vert)->get_outerFaces();
            if (find(vertexOuterFaces.begin(), vertexOuterFaces.end(), it)!=vertexOuterFaces.end())
            {
                v1Found=true;
                outerVertex1 = *it_vert;
            }
            it_vert++;
        }

        while (it_vert!=U_0.end() and !v2Found)
        {
            auto vertexOuterFaces = (*it_vert)->get_outerFaces();
            if (find(vertexOuterFaces.begin(), vertexOuterFaces.end(), it)!=vertexOuterFaces.end())
            {
                v2Found=true;
                outerVertex2 = *it_vert;
            }
            it_vert++;
        }

        // std::cout << "Vertex 1: " << outerVertex1 << ", vertex 2: " << outerVertex2 << '\n';

        newOuterEdge = std::make_shared<OuterEdge>(outerVertex1, outerVertex2);
        newOuterEdge->init();
        m_outerEdges.push_back(newOuterEdge);
    }

}

void StabilityPolytope::updateSupportFunctions(std::shared_ptr<Face>& dirFace)
{

    std::list<std::shared_ptr<Face>> consideredFaces;


    // need to init with a first few considered faces
    if (!dirFace.unique())
    {
        consideredFaces.push_back(dirFace);
    }

    consideredFaces.push_back(m_faces.back());

    auto currentFace = consideredFaces.begin();

    while (currentFace != consideredFaces.end())
    {
        // m_innerOuterLink[it_face->get_vertex1()]; -> gives an outer face but don't give a starting point
        auto initPoint = m_outerVertices.at(0);
        // std::cout << "Current Support Function: " << (*currentFace)->get_supportFunction() <<'\n';

        if (computeSupportFunction(*currentFace, initPoint))
        {
            // add neighbors
            auto currentNeighbors = (*currentFace)->findNeighbors();
            for (auto it : currentNeighbors)
            {
                if (find(consideredFaces.begin(), consideredFaces.end(), it)==consideredFaces.end())
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

bool StabilityPolytope::computeSupportFunction(std::shared_ptr<Face>& face, const std::shared_ptr<OuterVertex>& initPoint)
{
    // std::cout << "Computing support function for face " << face->get_index() << '\n';
    double prevSupportFunction = face->get_supportFunction();
    auto currentOuterVertex = initPoint;
    double currentDistance = face->get_normal().dot(currentOuterVertex->get_coordinates())-face->get_offset();
    double distance = 100;

    std::vector<std::shared_ptr<OuterVertex>> visitedPoints;
    visitedPoints.push_back(currentOuterVertex);

    bool currentIsSupport = false;

    while (!currentIsSupport)
    {
        currentIsSupport = true;
        auto currentNeighbors = currentOuterVertex->findNeighbors();

        for (auto it_v: currentNeighbors)
        {
            if (find(visitedPoints.begin(), visitedPoints.end(),it_v)==visitedPoints.end())
            {
                visitedPoints.push_back(it_v);
                distance = face->get_normal().dot(it_v->get_coordinates())-face->get_offset();
                if (distance > currentDistance)
                {
                    currentDistance = distance;
                    currentOuterVertex = it_v;
                    currentIsSupport = false;
                }
            }
        }
    }
    face->set_supportPoint(currentOuterVertex);

    if (abs(prevSupportFunction-face->get_supportFunction())<0.00000001)
    {
        return false;
    }
    else
    {
        // m_residual += face->get_area()*(face->get_supportFunction()-prevSupportFunction);
        return true;
    }
}

double StabilityPolytope::computeResidualFromScratch()
{
    m_residual = 0;
    for (auto it: m_faces)
    {
        m_residual += it->get_area()*it->get_supportFunction();
    }
}

bool StabilityPolytope::stopCriterion()
{
    return m_numberOfIterations < m_maxNumberOfIteration && m_residual > 0.1;
}

// ----------- output and display functions ----------
void StabilityPolytope::exportVertices(std::string file_name)
{

  std::ofstream file_stream(file_name);

    if (file_stream)
    {

        for (auto it_vertices : m_vertices)
        {
            file_stream << "iv;" // iv = inner vertices
                        << it_vertices->get_coordinates()[0] << ';'
                        << it_vertices->get_coordinates()[1] << ';'
                        << it_vertices->get_coordinates()[2] << ';'
                        << it_vertices->get_direction()[0] << ';'
                        << it_vertices->get_direction()[1] << ';'
                        << it_vertices->get_direction()[2] << ';' << std::endl;
        }

        for (auto it : m_edges)
        {
            file_stream << "ie;" // ie = inner edge
                        << it->get_vertex1()->get_coordinates()[0] << ';'
                        << it->get_vertex1()->get_coordinates()[1] << ';'
                        << it->get_vertex1()->get_coordinates()[2] << ';'
                        << it->get_vertex2()->get_coordinates()[0] << ';'
                        << it->get_vertex2()->get_coordinates()[1] << ';'
                        << it->get_vertex2()->get_coordinates()[2] << ';' << std::endl;
        }

        for (auto const& it : m_outerVertices)
        {
            file_stream << "ov;" // oe = outer vertex
                        << it->get_coordinates()[0] << ';'
                        << it->get_coordinates()[1] << ';'
                        << it->get_coordinates()[2] << ';' << std::endl;
        }

        for (auto const& it : m_outerEdges)
        {
            file_stream << "oe;" // oe = outer edge
                        << it->get_outerVertex1()->get_coordinates()[0] << ';'
                        << it->get_outerVertex1()->get_coordinates()[1] << ';'
                        << it->get_outerVertex1()->get_coordinates()[2] << ';'
                        << it->get_outerVertex2()->get_coordinates()[0] << ';'
                        << it->get_outerVertex2()->get_coordinates()[1] << ';'
                        << it->get_outerVertex2()->get_coordinates()[2] << ';' << std::endl;
        }
    }
    else
    {
        std::cerr << "Error: Impossible to open the output file." << '\n';
    }

    // glp_write_lp(m_lp, NULL, "export_lp.txt");
}

void StabilityPolytope::showPoly()
{
    std::cout << "Current Vertices: ";
    for (auto it : m_vertices)
    {
        std::cout << it->get_index() << ", ";
    }
    std::cout << '\n';

    std::cout << "Current Edges: " << '\n';
    for (auto it : m_edges)
    {
        std::cout << "Edge: " << it->get_index() << " has vertex " << it->get_vertex1()->get_index()
                  << " and "<< it->get_vertex2()->get_index() << '\n';
    }

    std::cout << '\n';
}

// ----------- getters ----------
int StabilityPolytope::get_numberOfVertices() const
{
    return m_vertices.size();
}

int StabilityPolytope::get_numberOfFaces() const
{
    return m_faces.size();
}

int StabilityPolytope::get_numberOfOuterVertices() const
{
    return m_outerVertices.size();
}

int StabilityPolytope::get_numberOfOuterFaces() const
{
    return m_outerFaces.size();
}

double StabilityPolytope::get_lpMicro() const
{
    return m_lpMicro;
}

double StabilityPolytope::get_innerConvexMicro() const
{
    return m_innerConvexMicro;
}

double StabilityPolytope::get_outerConvexMicro() const
{
    return m_outerConvexMicro;
}

double StabilityPolytope::get_supportFunctionMicro() const
{
    return m_supportFunctionMicro;
}

std::vector<Eigen::Vector3d> StabilityPolytope::get_innerFaceNormals() const
{
    std::vector<Eigen::Vector3d> innerFaceNormals;
    for (auto face: m_faces)
    {
        innerFaceNormals.push_back(face->get_normal());
    }
    return innerFaceNormals;
}

std::vector<double> StabilityPolytope::get_innerFaceOffsets() const
{
    std::vector<double> innerFaceOffsets;
    for (auto face: m_faces)
    {
        innerFaceOffsets.push_back(face->get_offset());
    }
    return innerFaceOffsets;
}

ContactSet* StabilityPolytope::get_contactSet()
{
    return &m_contactSet;
}

Solver StabilityPolytope::get_solver() const
{
  return m_solver;
}
// ------------------ setter -----------------------

void StabilityPolytope::set_maxNumberOfIterations(int maxNumberOfIteration)
{
    m_maxNumberOfIteration = maxNumberOfIteration;
}
