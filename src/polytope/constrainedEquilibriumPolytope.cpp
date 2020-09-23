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
  polyMax_->initSolver();
  polyMax_->projectionStabilityPolyhedron();
  polyMax_->endSolver();
  // projection polMin
  polyMin_->initSolver();
  polyMin_->projectionStabilityPolyhedron();
  polyMin_->endSolver();

  m_solverEnded = true;

  // // get the planes and vertices from the inner approximation of polMax
  auto maxPlanes = polyMax_->constraintPlanes();
  auto maxVertices = polyMax_->vertices();

  // // get the planes and vertices from the inner approximation of polMin
  auto minPlanes = polyMin_->constraintPlanes();
  auto minVertices = polyMin_->vertices();
   
  std::vector<Eigen::Vector3d> vertices;
  vertices.insert(vertices.end(), maxVertices.begin(), maxVertices.end());
  vertices.insert(vertices.end(), minVertices.begin(), minVertices.end());

  /* to compute the intersection I have two solutions:
   * 1- Use QHull to compute it directly
   * 2- Take the points in min that are in max and take the point in max that are in min

   * First solution seems to be the best
   * Second Solution is tricky and should be last resort
   */ 


  
  // // compute the intersection of polMax and polMin using qhull
  // // planes_.insert(planes_.end(), maxPlanes.begin(), maxPlanes.end());
  std::vector<Eigen::Vector4d> planes;
  planes.insert(planes.end(), maxPlanes.begin(), maxPlanes.end());
  // planes.insert(planes.end(), minPlanes.begin(), minPlanes.end());
  planes_ = {};
  planes_ = planes;
  
  // running Qhull
  // std::vector<double> planes_coord;
  
  // std::cout << "List of the planes: " << std::endl;
  // for (auto plane: planes)
  //   {
  //     planes_coord.push_back(plane[0]);
  //     planes_coord.push_back(plane[1]);
  //     planes_coord.push_back(plane[2]);
  //     planes_coord.push_back(-plane[3]);
  //     std::cout << plane.transpose() << std::endl;
  //   }
  // std::cout << "End of the list!" << std::endl;

  // std::vector<double> ptsCoord;
  // for (auto v:vertices)
  //   {
  //     ptsCoord.push_back(v[0]);
  //     ptsCoord.push_back(v[1]);
  //     ptsCoord.push_back(v[2]);
  //   }
  
  // // std::cout << "Starting Qhull" << std::endl;
  // Eigen::Vector3d feasiblePt = StabilityPolytope::chebichevCenter(planes);
  // std::cout << "[Stabiliplus] feasible Point is: " << feasiblePt.transpose() << std::endl;
  
  // orgQhull::Qhull qhull;
  // // qhull.setOutputStream(&std::cout);
  // // qhull.enableOutputStream();
  
  // std::vector<double> fpCoords;
  // fpCoords.push_back(feasiblePt[0]);
  // fpCoords.push_back(feasiblePt[1]);
  // fpCoords.push_back(feasiblePt[2]);
  // orgQhull::Coordinates feasiblePoint(fpCoords);
  
  
  // qhull.runQhull("", 4, planes.size(), planes_coord.data(), "H Fp Fv");
  // qhull.runQhull("", 3, vertices.size(), ptsCoord.data(), "s n");

  // I guess it is time for some display
  
  // auto facetList = qhull.facetList().toStdVector();
  // // store in planes_ the facets
  // Eigen::Vector4d plane;

  // //std::cout << "List of the planes according to QHull:" << std::endl;
  // for (auto facet: facetList)
  // {
  //   auto innerPlane = facet.hyperplane();
  //   plane << innerPlane[0],
  //     innerPlane[1],
  //     innerPlane[2],
  //     -innerPlane.offset();
  //   // plane << *(facet.hyperplane().coordinates()),
  //   //   *(facet.hyperplane().coordinates()+1),
  //   //   *(facet.hyperplane().coordinates()+2),
  //   //   -facet.hyperplane().offset();

  //   planes_.push_back(plane);
  // }
  //std::cout << "End of the list of planes " << std::endl;
    
  
  // auto qhullVertices = qhull.vertexList().toStdVector();
  // // store in vertices_ the vertices
  
  // Eigen::Vector3d vertex;
  // int index = 0;
  
  // for (auto v: qhullVertices)
  //   {
  //     vertex << *(v.point().coordinates()),
  // 	*(v.point().coordinates()+1),
  // 	*(v.point().coordinates()+2);
  //     vertices_[index] = vertex;
  //     index ++;
  //   }
  
  // //std::cout << qhull.facetList();
  
  // vertices_.erase(vertices_.begin(), vertices_.end());
  // orgQhull::QhullHyperplane hyperplane;
  // Eigen::Vector3d coord;
  // double offset;

  // // std::cout << "Qhull computations done" << std::endl;
  // // std::cout << "Extrating the results" << std::endl;
  
  // for (auto facet: facetList)
  //   {
  //     for (auto face: facet.neighborFacets().toStdVector())
  // 	{
  // 	  // if the pair is not already in the edge list then add it

  // 	  auto test = [this](int f1, int f2){

  // 	    for (auto e:edges_)
  // 	      {
  // 		if (e.first==f1 and e.second==f2)
  // 		  {
  // 		    return false;
  // 		  }
  // 		if (e.first==f2 and e.second==f1)
  // 		  {
  // 		    return false;
  // 		  }
  // 	      }
	      
  // 	    return true;
  // 	  };
	  
  // 	  if (test(facet.id(), face.id()))
  // 	    {
  // 	      edges_.emplace_back(facet.id(), face.id());
  // 	      //std::cout << "Edge: " << facet.id() << ", " << face.id() << std::endl;
  // 	    }
  // 	}
      
  //     // std::cout << vert.dimension() << std::endl;
  //     hyperplane = facet.hyperplane();
      
  //     offset = hyperplane.offset();
  //     coord[0] = fpCoords[0]-*(hyperplane.coordinates())/offset;
  //     coord[1] = fpCoords[1]-*(hyperplane.coordinates()+1)/offset;
  //     coord[2] = fpCoords[2]-*(hyperplane.coordinates()+2)/offset;
      
  //     vertices_[facet.id()]=coord;
  //   }
  //   // extract also only the usefull planes...

  // auto qhullVertices = qhull.vertexList().toStdVector();

  // for (auto v: qhullVertices)
  //   {
  //     planes_.push_back(planes.at(v.point().id()));
  //   }

  // std::cout << "Results Extracted" << std::endl;
  // std::cout << "Projected ChebichevCenter: " << projectChebMaxOnPolyMin().transpose() << std::endl;

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
            
  //     for (auto edge: edges_)
  // 	{
  // 	  v1 = vertices_.at(edge.first);
  // 	  v2 = vertices_.at(edge.second);
	  
  // 	  stream << "ie;"
  // 		 << v1[0] << ";" << v1[1] << ";" << v1[2] << ";"
  // 		 << v2[0] << ";" << v2[1] << ";" << v2[2] << ";"
  // 		 << std::endl;
  // 	}
    }
  else
    {
      std::cerr << "Error: Output Stream not open." << std::endl;
    }
}

tinyxml2::XMLElement * ConstrainedEquilibriumPolytope::xmlPolytope(tinyxml2::XMLDocument & doc) const
{
  auto xmlPoly = doc.NewElement("polytope");
  xmlPoly->SetAttribute("type", "constrained");

  xmlPoly->InsertEndChild(polyMax_->xmlPolytope(doc));
  xmlPoly->InsertEndChild(polyMin_->xmlPolytope(doc));

  return xmlPoly;
}

void ConstrainedEquilibriumPolytope::save(std::string name) const
{
  std::string maxName = name + "_max.txt";
  std::string minName = name + "_min.txt";
  std::ofstream  maxStream(maxName), minStream(minName);

  polyMax_->writeToStream(maxStream);
  polyMin_->writeToStream(minStream);
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

  baryPt = polyMin_->baryPoint()+polyMax_->baryPoint();
  baryPt /= 2;

  return baryPt;
}

Eigen::Vector3d ConstrainedEquilibriumPolytope::chebichevCenter() const
{
  // Eigen::Vector3d center = Eigen::Vector3d::Zero();

  // center = polyMin_->chebichevCenter() + polyMax_->chebichevCenter();
  // center /= 2;

  // return center;
  //projectChebMaxOnPolyMin();
  
  return StabilityPolytope::chebichevCenter(planes_);
}

int ConstrainedEquilibriumPolytope::get_numberOfVertices() const
{
  return vertices_.size();
}

// Eigen::Vector3d ConstrainedEquilibriumPolytope::projectChebMaxOnPolyMin() const
// {
  
//   // create the sch-core S_Polyhedron object

//   sch::S_Polyhedron poly;
//   auto polyAlgo = poly.getPolyhedronAlgorithm();
  
//   /*
//     First add the vertices:
//     - get them from the poly object
//     - create a S_PolyhedronVertex Object 
//     - add it the S_Polyhedron object using the right method
//   */

//   sch::S_PolyhedronVertex *v;
//   Eigen::Vector3d coord;
  
//   for (auto vertex: polyMin_->fullVertices())
//     {
//       v = new sch::S_PolyhedronVertex();
//       coord = vertex->get_coordinates();
//       v->setCoordinates(coord[0], coord[1], coord[2]);
//       v->setNumber(unsigned (vertex->get_index()));
//       polyAlgo->vertexes_.push_back(v);
//     }
  
//   /*
//     Second add the normal/faces
//     - Get the faces from the polytope object
//     - create the corresponding PolyhedronTriangle Object
//         - normal 
// 	- corresponding vertices
//     - add it to the S_Polyhedron

//     Third add the neighboring vertices
//     - Go through the edges of the polytope object 
//     - Or add them when creating the triangles
//   */

//   int a, b, c;
//   sch::S_PolyhedronVertex *va, *vb, *vc;
  
//   for (auto face: polyMin_->fullFaces())
//     {

//       sch::PolyhedronTriangle t;
//       coord = face->get_normal();

//       t.normal.Set(coord[0], coord[1], coord[2]);
//       t.normal.normalize();

//       a = face->get_vertex1()->get_index();
//       b = face->get_vertex2()->get_index();
//       c = face->get_vertex3()->get_index();

//       auto pred_a = [&a](sch::S_PolyhedronVertex *v){
// 	return v->getNumber() == a;
//       };
//       va = *(std::find_if(polyAlgo->vertexes_.begin(), polyAlgo->vertexes_.end(), pred_a));

//       auto pred_b = [&b](sch::S_PolyhedronVertex *v){
// 	return v->getNumber() == b;
//       };
//       vb = *(std::find_if(polyAlgo->vertexes_.begin(), polyAlgo->vertexes_.end(), pred_b));

//       auto pred_c = [&c](sch::S_PolyhedronVertex *v){
// 	return v->getNumber() == c;
//       };
//       vc = *(std::find_if(polyAlgo->vertexes_.begin(), polyAlgo->vertexes_.end(), pred_c));

//       va->addNeighbor(vb);
//       va->addNeighbor(vc);

//       vb->addNeighbor(va);
//       vb->addNeighbor(vc);

//       vc->addNeighbor(va);
//       vc->addNeighbor(vb);
      
//       polyAlgo->triangles_.push_back(t);
//     }
  
//   /*
//     Forth apply the finishing method
//     - updateFastArrays
//     - deleteVertexesWithoutNeighbors
//    */
//   polyAlgo->updateVertexNeighbors();
//   polyAlgo->updateFastArrays();
  
//   // project
//   return Eigen::Vector3d::Zero();
// }
