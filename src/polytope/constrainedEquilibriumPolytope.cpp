
#include "polytope/constrainedEquilibriumPolytope.h"


ConstrainedEquilibriumPolytope::ConstrainedEquilibriumPolytope(std::shared_ptr<ContactSet> contactSet, int maxIteration, double maxError, Solver solverType):
  StabilityPolytope::StabilityPolytope( contactSet, maxIteration, maxError, solverType)
{
  //get the list of constrained contacts
  constrainedContactNames_ = contactSet->constrainedContactNames();

  // create a copy of the contactSet contactMax with fmax assigned to the constrained contacts
  contactSetMax_ = std::make_shared<ContactSet> (*contactSet);
  // create a copy of the contactSet contactMin with fmin assigned to the constrained contacts
  contactSetMin_ = std::make_shared<ContactSet> (*contactSet);

  // get fmin and fmax
  double fmax, fmin;

  for (auto contactName: constrainedContactNames_)
    {
      fmax = contactSet->contactFMax(contactName);
      fmin = contactSet->contactFMin(contactName);

      constrainedContactFmax_.push_back(fmax);
      constrainedContactFmin_.push_back(fmin);

      contactSetMax_->setContactFMin(fmax, contactName);
      contactSetMin_->setContactFMax(fmin, contactName);
    }


  // create a stability region polMax with contactMax
  polyMax_ = std::make_shared<RobustStabilityPolytope> (contactSetMax_, maxIteration, maxError, solverType);
  // create a stability region polMin with contactMin
  polyMin_ = std::make_shared<RobustStabilityPolytope> (contactSetMin_, maxIteration, maxError, solverType);
}

void ConstrainedEquilibriumPolytope::initSolver()
{
  // init polMax
  
  // init polMin
  

}

void ConstrainedEquilibriumPolytope::projectionStabilityPolyhedron()
{
  // projection polMax
  // std::cout << "Projection of the Min Polytope" << std::endl;
  polyMax_->initSolver();
  polyMax_->projectionStabilityPolyhedron();
  polyMax_->endSolver();
  // projection polMin
  // std::cout << "Projection of the Max Polytope" << std::endl;
  polyMin_->initSolver();
  // std::cout << "Did I stop here?" << std::endl;
  polyMin_->projectionStabilityPolyhedron();
  // std::cout << "Did I stop here?" << std::endl;
  polyMin_->endSolver();

  m_solverEnded = true;
  // std::cout << "Projections finished" << std::endl;
  
  // get the planes and vertices from the inner approximation of polMax
  auto maxPlanes = polyMax_->constraintPlanes();
  auto maxVertices = polyMax_->vertices();
  // get the planes and vertices from the inner approximation of polMin
  auto minPlanes = polyMin_->constraintPlanes();
  auto minVertices = polyMin_->vertices();
  
  // compute the intersection of polMax and polMin using qhull
  // planes_.insert(planes_.end(), maxPlanes.begin(), maxPlanes.end());
  std::vector<Eigen::Vector4d> planes;
  planes.insert(planes.end(), maxPlanes.begin(), maxPlanes.end());
  planes.insert(planes.end(), minPlanes.begin(), minPlanes.end());

  // running Qhull
  std::vector<double> planes_coord;
  
  // std::cout << "List of the planes: " << std::endl;
  for (auto plane: planes)
    {
      planes_coord.push_back(plane[0]);
      planes_coord.push_back(plane[1]);
      planes_coord.push_back(plane[2]);
      planes_coord.push_back(-plane[3]);    
    }
  // std::cout << "End of the list!" << std::endl;

  // std::cout << "Starting Qhull" << std::endl;
  Eigen::Vector3d feasiblePt = chebichevCenter(planes);
  
  orgQhull::Qhull qhull;

  std::vector<double> fpCoords;
  fpCoords.push_back(feasiblePt[0]);
  fpCoords.push_back(feasiblePt[1]);
  fpCoords.push_back(feasiblePt[2]);
  orgQhull::Coordinates feasiblePoint(fpCoords);
  qhull.setFeasiblePoint(feasiblePoint);
  
  qhull.runQhull("", 4, planes.size(), planes_coord.data(), "H");

  auto facetList = qhull.facetList().toStdVector();
  //std::cout << qhull.facetList();
  
  vertices_.erase(vertices_.begin(), vertices_.end());
  orgQhull::QhullHyperplane hyperplane;
  Eigen::Vector3d coord;
  double offset;

  // std::cout << "Qhull computations done" << std::endl;
  // std::cout << "Extrating the results" << std::endl;
  
  for (auto facet: facetList)
    {
      for (auto face: facet.neighborFacets().toStdVector())
	{
	  // if the pair is not already in the edge list then add it

	  auto test = [this](int f1, int f2){

	    for (auto e:edges_)
	      {
		if (e.first==f1 and e.second==f2)
		  {
		    return false;
		  }
		if (e.first==f2 and e.second==f1)
		  {
		    return false;
		  }
	      }
	      
	    return true;
	  };
	  
	  if (test(facet.id(), face.id()))
	    {
	      edges_.emplace_back(facet.id(), face.id());
	      //std::cout << "Edge: " << facet.id() << ", " << face.id() << std::endl;
	    }
	}
      
      // std::cout << vert.dimension() << std::endl;
      hyperplane = facet.hyperplane();
      
      offset = hyperplane.offset();
      coord[0] = fpCoords[0]-*(hyperplane.coordinates())/offset;
      coord[1] = fpCoords[1]-*(hyperplane.coordinates()+1)/offset;
      coord[2] = fpCoords[2]-*(hyperplane.coordinates()+2)/offset;
      
      vertices_[facet.id()]=coord;
    }
    // extract also only the usefull planes...

  auto qhullVertices = qhull.vertexList().toStdVector();

  for (auto v: qhullVertices)
    {
      planes_.push_back(planes.at(v.point().id()));
    }

  //std::cout << "Results Extracted" << std::endl;
}

void ConstrainedEquilibriumPolytope::endSolver()
{
  // polyMax_->endSolver();
  // polyMin_->endSolver();
}

void ConstrainedEquilibriumPolytope::writeToStream(std::ofstream & stream) const
{
  // save the vertices and planes
  // polyMax_->writeToStream(stream);
  // polyMin_->writeToStream(stream);
  // save the points
  Eigen::Vector3d v1, v2;
  
  if (stream)
    {
      for (auto vert: vertices_)
  	{
	  v1 = vert.second;
  	  stream << "iv;"
  		 << v1[0] << ";" << v1[1] << ";" << v1[2] << ";"
  		 << 0.0 << ";" << 0.0 << ";" << 0.0 << ";" // could use the dual to define something here...
  		 << std::endl;
  	}
            
      for (auto edge: edges_)
	{
	  v1 = vertices_.at(edge.first);
	  v2 = vertices_.at(edge.second);
	  
	  stream << "ie;"
		 << v1[0] << ";" << v1[1] << ";" << v1[2] << ";"
		 << v2[0] << ";" << v2[1] << ";" << v2[2] << ";"
		 << std::endl;
	}
    }
  else
    {
      std::cerr << "Error: Output Stream not open." << std::endl;
    }
}



std::vector<Eigen::Vector4d> ConstrainedEquilibriumPolytope::constraintPlanes() const
{
  return planes_;
}

std::vector<Eigen::Vector3d> ConstrainedEquilibriumPolytope::vertices() const
{
  std::vector<Eigen::Vector3d> vertices;
  for (auto vert: vertices_)
    {
      vertices.push_back(vert.second);
    }
  return vertices;
}

bool ConstrainedEquilibriumPolytope::vertexInPlanes(Eigen::Vector3d vertex, std::vector<Eigen::Vector4d> planes, double eps) const
{
  Eigen::Vector3d normal;
  double offset;
  
  for ( auto plane: planes)
    {
      normal << plane(0), plane(1), plane(2);
      offset = plane(3);
      if (normal.transpose().dot(vertex)>offset-eps)
	{
	  return false;
	}
    }
  return true;
}

Eigen::Vector3d ConstrainedEquilibriumPolytope::baryPoint() const
{
  Eigen::Vector3d baryPt = Eigen::Vector3d::Zero();

  for(auto pt : vertices_)
  {
    baryPt += pt.second;
  }

  baryPt /= vertices_.size();

  return baryPt;
}

int ConstrainedEquilibriumPolytope::get_numberOfVertices() const
{
  return vertices_.size();
}
