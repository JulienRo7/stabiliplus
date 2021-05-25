#include "experimenter.h"

// class ComputationPoint 

ComputationPoint::ComputationPoint(std::string const & contactSetFileName, int numFrictionSides, Solver solver, bool robust): contactSetFileName_(contactSetFileName), numFrictionSides_(numFrictionSides), solver_(solver), robust_(robust)
{
  contactSet_ = std::make_shared<ContactSet> (!robust_, contactSetFileName_, numFrictionSides);
}

ComputationPoint::~ComputationPoint() {}

void ComputationPoint::compute() {
  auto start = std::chrono::high_resolution_clock::now();
  if (robust_)
    {
      // if (contactSet->hasConstrainedContact())
      // 	{
      // 	  std::cout << "Constrained Equilibrium Polytope!" << std::endl;
      // 	  polytope = std::make_shared<ConstrainedEquilibriumPolytope> (contactSet, 50, 0.05, m_solver);
      // 	}
      // else
      // 	{
      // 	  std::cout << "Robust Equilibrium Polytope!" << std::endl;
      // 	  polytope = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, m_solver);
      // 	}
      polytope_ = std::make_shared<RobustStabilityPolytope> (contactSet_, 50, 0.05, solver_);
      
    }
  else
    {
      polytope_ = std::make_shared<StaticStabilityPolytope> (contactSet_, maxIt_, precision_, solver_);
    }
  
  polytope_->initSolver();
  polytope_->projectionStabilityPolyhedron();
  
  auto stop = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop -start);

  polytope_->endSolver();
  
  totalTime_ = duration.count();

  // Compute the points that need to be saved using the lambda functions
  computeOptimQP();
  for (auto& cptPt: computerPoints_)
    {
      computedPoints_[cptPt.first] = cptPt.second(this);
    }
}

void ComputationPoint::display() const
{
  
}

void ComputationPoint::printTimings() const
{
  if (!robust_ and !contactSet_->hasConstrainedContact())
    {
      auto poly = std::dynamic_pointer_cast<RobustStabilityPolytope> (polytope_);
	
      std::cout << "Computation time: " << totalTime_ << "ms for " << poly->get_numberOfVertices() << " inner Vertices and "  << poly->get_numberOfOuterVertices() << " outer vertices." << std::endl;
      std::cout << "Number of Inner Faces: " << poly->get_numberOfFaces() << ", Number of outer faces: "  << poly->get_numberOfOuterFaces() << std::endl;
      std::cout << "LP time: " << poly->LPTime() << " µs"<< std::endl;
      std::cout << "inner time: " << poly->get_innerConvexMicro() << " µs" << std::endl;
      std::cout << "outer time: " << poly->get_outerConvexMicro() << " µs" << std::endl;
      std::cout << "support time: " << poly->get_supportFunctionMicro() << " µs" << std::endl;
    }
  else
    {
      std::cout << "Equilibrium  Region computed in " << totalTime_ << " ms for " << polytope_->get_numberOfVertices()
      		<< " inner Vertices" << std::endl;
      std::cout << "Init time: " << polytope_->initTime() << " µs" << std::endl;
      std::cout << "LP time: " << polytope_->LPTime() << " µs with solver " << solver_ << std::endl;
      std::cout << "Structure time: " << polytope_->structTime() << " µs" << std::endl;
    }
}

void ComputationPoint::computeOptimQP()
{
  comQP_ = std::make_shared<CoMQP>();
  comQP_->setContactSet(contactSet_);
  comQP_->dontConsiderConstrained();

  // objective CoM
  // Eigen::Vector3d com =  poly->chebichevCenter();
  Eigen::Vector3d com =  polytope_->baryPoint();
  com(2) = 0.75;
    
  comQP_->solve(com);

  // computing the contact forces.
  auto result = comQP_->resultVector();

  // the first 4x3 lines correspond to LF
  forceLF_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++)
    {
      forceLF_ += result.segment<3>(3*i);
    }
  
  // the next 4x3 lines correspond to RF
  forceRF_ = Eigen::Vector3d::Zero();
  for (int i = 0; i<4; i++)
    {
      forceRF_ += result.segment<3>(4*3+3*i);
    }
  
  // if there are more then 4x3 + 4x3 + 3 variable the next 5x3 variables correspond to RH
  forceRH_ = Eigen::Vector3d::Zero();
  if (result.size() > 27)
    {
      for (int i = 0; i<5; i++)
	{
	  forceRH_ += result.segment<3>(24 + 3 * i);
	}
    }
}

Eigen::Vector3d ComputationPoint::getOptimCoM() const
{
  auto com = comQP_->resultCoM();
  std::cout << "The QP is solved, the result is " << com.transpose() << std::endl;
  return com;
}


void ComputationPoint::addLambda(std::string name, std::function<Eigen::Vector3d(ComputationPoint*)> computer, std::string color)
{
  computerPoints_[name]=computer;
  computedPointsColor_[name]=color;
}

tinyxml2::XMLElement * ComputationPoint::xmlComputationPoint(tinyxml2::XMLDocument & doc, int index) const
{  
  // get index from name? maybe...
  
  // Saving the polytope: this should depend on the type of polytope (constrained Equilibrium need to save two polytopes) Maybe I should consider saving polytopes as xml...
  std::string poly_name;
  poly_name = "/tmp/polytopes/polytope_" + std::to_string(index) + ".xml";
  //std::ofstream stream(poly_name);
  polytope_->saveToFile(poly_name);
  //stream.close();

  // There is no need to save the contact Set as it is the input of the problem, I can just give the name of the file.
  // if(typeid(ContactSet) == typeid(*(poly->problemDescriptor())))
  // {
  //   auto csPtr = static_cast<ContactSet *>(poly->problemDescriptor().get());
  //   csPtr->saveContactSet(robot_file_name);
  // }

  tinyxml2::XMLElement * xmlComputationPoint = doc.NewElement("compPoint");
  xmlComputationPoint->SetAttribute("index", index);
  // root->InsertEndChild(compPoint);

  tinyxml2::XMLElement * polyXML = doc.NewElement("poly");
  polyXML->SetAttribute("file_name", poly_name.c_str());
  xmlComputationPoint->InsertEndChild(polyXML);

  tinyxml2::XMLElement * robotXML = doc.NewElement("robot");

  // if(typeid(ContactSet) == typeid(*(poly->problemDescriptor())))
  // {

  //   auto csPtr = static_cast<ContactSet *>(poly->problemDescriptor().get());
  //   csPtr->saveContactSet(robot_file_name);

  //   robotXML->SetAttribute("name", csPtr->get_name().c_str());
  // }
  robotXML->SetAttribute("name", contactSet_->get_name().c_str());
  robotXML->SetAttribute("file_name", contactSetFileName_.c_str());
  xmlComputationPoint->InsertEndChild(robotXML);

  tinyxml2::XMLElement * timeXML = doc.NewElement("times");
  timeXML->SetAttribute("total", totalTime_);
  timeXML->SetAttribute("LP", polytope_->LPTime());
  timeXML->SetAttribute("init", polytope_->initTime());
  timeXML->SetAttribute("struct", polytope_->structTime());
  // timeXML->SetAttribute("inner", polytope_->innerConvexMicro());
  // timeXML->SetAttribute("outer", polytope_->outerConvexMicro());
  // timeXML->SetAttribute("support", polytope_->supportFunctionMicro());
  xmlComputationPoint->InsertEndChild(timeXML);

  tinyxml2::XMLElement * solverXML = doc.NewElement("solver");
  switch(solver_)
    {
    case GLPK:
      solverXML->SetAttribute("name", "GLPK");
      break;

    case LP_SOLVE:
      solverXML->SetAttribute("name", "LP_SOLVE");
      break;

    case GUROBI:
      solverXML->SetAttribute("name", "GUROBI");
      break;
    }
  xmlComputationPoint->InsertEndChild(solverXML);

  // add the points here

  // if robust

  auto addPoint = [&doc, &xmlComputationPoint](Eigen::Vector3d coord, std::string name, std::string color){
    auto ptXML = doc.NewElement("point");
    ptXML->SetAttribute("name", name.c_str());

    ptXML->SetAttribute("x", coord[0]);
    ptXML->SetAttribute("y", coord[1]);
    ptXML->SetAttribute("z", coord[2]);

    ptXML->SetAttribute("color", color.c_str());

    xmlComputationPoint->InsertEndChild(ptXML);
  };

  Eigen::Vector3d point;

  for (auto& cptPt: computedPoints_)
    {
      addPoint(cptPt.second, cptPt.first, computedPointsColor_.at(cptPt.first));
    }
    
  return xmlComputationPoint;
}

// ---------- constructors and destructor -----------

Experimenter::Experimenter(int mode,
                           std::string const & contact_set_file_name,
                           int numFrictionSides,
                           Solver solver,
                           bool robust)
  : m_mode(mode), m_contactSetFileName(contact_set_file_name),
    m_numFrictionSides(numFrictionSides), m_solver(solver),
    m_robust(robust) 
{
}

Experimenter::~Experimenter() {}

// ---------- main functions -----------

void Experimenter::run()
{
  std::cout << "Running experiment!" << '\n';
  switch(m_mode)
    {
    case 1:
      run_exp1();
      break;
    // case 2:
    //   run_exp2();
    //   break;
    // case 3:
    //   run_exp3();
    //   break;
    // case 4:
    //   run_exp4();
    //   break;
    case 5:
      run_exp5();
      break;
    case 6:
      run_exp6();
      break;
    default:
      std::cerr << "Unknown mode" << '\n';
  }
  std::cout << "Experiment done!" << '\n';
}

void Experimenter::run_exp1()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running Experiment for mode 1!" << '\n';

  std::shared_ptr<ComputationPoint> compPt;
  compPt = std::make_shared<ComputationPoint> (m_contactSetFileName, m_numFrictionSides, m_solver, m_robust);

  computationPoints_.push_back(compPt);
  
  compPt->compute();
  // compPt->printTimings();
      
      
  
}

// void Experimenter::run_exp2()
// {
//   std::cout << "#-----------------------------" << std::endl;
//   std::cout << "Running Experiment for mode 2!" << '\n';

//   std::string robot_names[4] = {"./robots/robot_1.xml", "./robots/robot_2.xml", "./robots/robot_3.xml",
//                                 "./robots/robot_4.xml"};
//   int numTrials = 100;
//   //Solver solvers[3] = {GLPK, LP_SOLVE, GUROBI};
//   Solver solvers[1] = {GLPK};

//   bool staticCase = !m_robust;

//   for(auto solver : solvers)
//   {
//     for(auto rob_file : robot_names)
//     {
//       std::shared_ptr<ContactSet> rob = std::make_shared<ContactSet>(staticCase, rob_file, m_numFrictionSides);
//       for(int i = 0; i < numTrials; i++)
//       {
//         std::cout << "Solver: " << solver << " ContactSet: " << rob_file << " Run: " << i + 1 << '/' << numTrials
//                   << '\n';
//         auto start = std::chrono::high_resolution_clock::now();

// 	std::shared_ptr<StabilityPolytope> polytope;
// 	if(m_robust)
// 	  {
// 	    polytope = std::make_shared<RobustStabilityPolytope> (rob, 50, 0.05, solver);
// 	  }
// 	else
// 	  {
// 	    polytope = std::make_shared<StaticStabilityPolytope> (rob, 50, 0.01, solver);
// 	  }
        

//         polytope->initSolver();
//         polytope->projectionStabilityPolyhedron();

//         auto stop = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

// 	m_contactSets.push_back(rob);
//         m_polytopes.push_back(polytope);
//         m_total_times.push_back(duration.count());
//       }
//     }
//   }  
// }

// void Experimenter::run_exp3()
// {
//   std::cout << "#-----------------------------" << std::endl;
//   std::cout << "Running Experiment for mode 3!" << '\n';

//   std::shared_ptr<ContactSet> contactSet;
//   std::shared_ptr<StabilityPolytope> polytope;

//   Eigen::Vector3d dx;
//   dx << 0.0, 0.01, 0.0;
  
//   // if(contactSet->get_name() == "robot_8")
//   // {
//   for(int i = 0; i < 50; i++)
//     {
//       contactSet = std::make_shared<ContactSet>(!m_robust, m_contactSetFileName, m_numFrictionSides);
//       contactSet->translateContact(3, i*dx);
      
//       auto start = std::chrono::high_resolution_clock::now();
      
//       if(m_robust)
// 	{
// 	  polytope = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, m_solver);
// 	}
//       else
// 	{
// 	  polytope = std::make_shared<StaticStabilityPolytope> (contactSet, 50, 0.01, m_solver);
// 	}

//       polytope->initSolver();
//       polytope->projectionStabilityPolyhedron();

//       auto stop = std::chrono::high_resolution_clock::now();
//       auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

//       m_contactSets.push_back(contactSet);
//       m_polytopes.push_back(polytope);
//       m_total_times.push_back(duration.count());
//     }
//   // }
//   // else
//   // {
//   //   std::cerr << "This experiment requires robot 8 to be loaded!" << '\n';
//   // }
  
// }

// void Experimenter::run_exp4()
// {
//   std::cout << "#-----------------------------" << std::endl;
//   std::cout << "Running experiment for mode 4!" << std::endl;

//   std::shared_ptr<ContactSet> contactSet;
  
//   std::string contactName("contact_exp4");
  
//   Eigen::Matrix4d homTrans;
//   homTrans << 1, 0, 0, 1,
//     0, 1, 0, 1,
//     0, 0, 1, 0,
//     0, 0, 0, 1;

//   double fmax(10);
//   double df = fmax/20;
//   int n_pts(60);

//   auto f = [&] (int i) {
//     if (i <= 20)
//       {
// 	return 0.;
//       }
//     else
//       {
// 	if (i>=40)
// 	  {
// 	    return fmax;
// 	  }
// 	else
// 	  {
// 	    return (i-20)*df;
// 	  }
//       }
//   };

  
//   for (int i=0; i<= n_pts; i++)
//     {
//       // Load the contact set
//       contactSet = std::make_shared<ContactSet> (!m_robust, m_contactSetFileName, m_numFrictionSides);
  
//       // Add a contact point with fmax to 0 
//       contactSet->addContact(contactName, homTrans, 0.5, f(i), 0);

//       computePoint(contactSet);
      
//       m_contactSets.push_back(contactSet);
//     }
// }


void Experimenter::run_exp5()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running experiment for mode 5!" << std::endl;
  std::cout << std::endl;

  std::cout << "Loading files from folder: " << m_contactSetFileName << std::endl;
  
  std::vector<std::string> contactSetNames;
  
  for (auto p: std::filesystem::directory_iterator(m_contactSetFileName))
    {
      contactSetNames.push_back(p.path().string());
    }

  auto grabNum = [](std::string file){
    int under = file.find("_")+1;
    int dot = file.find(".");
    return std::stoi(file.substr(under, dot-under));
  };

  auto compFiles = [grabNum](std::string file1, std::string file2){
    return grabNum(file1) < grabNum(file2);
  };
  
  std::sort(contactSetNames.begin(), contactSetNames.end(), compFiles);

  std::cout << "Found "<< contactSetNames.size() << " contactSet files" << std::endl;

  // creating the lambda functions
  auto computerBaryPoint = [](ComputationPoint * comptPt){
    auto poly = comptPt->polytope();
    return poly->baryPoint();
  };

  auto computerChebichev = [](ComputationPoint * comptPt){
    auto poly = comptPt->polytope();
    return poly->chebichevCenter();
  };

  // auto computerBaryPointMin = [](ComputationPoint * comptPt){
  //   auto poly = std::dynamic_pointer_cast<ConstrainedEquilibriumPolytope> (comptPt->polytope());
  //   return poly->baryPointMin();
  // };

  // auto computerBaryPointMax = [](ComputationPoint * comptPt){
  //   auto poly = std::dynamic_pointer_cast<ConstrainedEquilibriumPolytope> (comptPt->polytope());
  //   return poly->baryPointMax();
  // };

  // auto computerChebichevMin = [](ComputationPoint * comptPt){
  //   auto poly = std::dynamic_pointer_cast<ConstrainedEquilibriumPolytope> (comptPt->polytope());
  //   return poly->chebichevCenterMin();
  // };
  
  // auto computerChebichevMax = [](ComputationPoint * comptPt){
  //   auto poly = std::dynamic_pointer_cast<ConstrainedEquilibriumPolytope> (comptPt->polytope());
  //   return poly->chebichevCenterMax();
  // };  

  auto computerCoMQP = [](ComputationPoint * comptPt){
    return comptPt->getOptimCoM();
  };

  // auto computerCoMQPProjected = [](ComputationPoint * comptPt){
        
  //   auto contactSet = comptPt->contactSet();
  //   auto comQP = CoMQP(contactSet);

  //   auto poly = std::dynamic_pointer_cast<ConstrainedEquilibriumPolytope> (comptPt->polytope());
  //   Eigen::Vector3d com = poly->chebichevCenterMax();
  //   com(2) = 0.75;
    
  //   comQP.solve(com);
    
  //   return comQP.resultCoM();
  // };

  auto forceLF = [](ComputationPoint * comptPt){
    return comptPt->getForceLF();
  };

  auto forceRF = [](ComputationPoint * comptPt){
    return comptPt->getForceRF();
  };

  auto forceRH = [](ComputationPoint * comptPt){
    return comptPt->getForceRH();
  };
  
  // creating the ComputationPoint obejcts
  std::shared_ptr<ComputationPoint> compPt;
  
  for (auto name: contactSetNames)
    {
      compPt = std::make_shared<ComputationPoint> (name, m_numFrictionSides, m_solver, m_robust);
      // if (compPt->contactSet()->hasConstrainedContact())
      // 	{
      // 	  // compPt->addLambda("baryPoint_Min", computerBaryPointMin, "xkcd:red");
      // 	  // compPt->addLambda("baryPoint_Max", computerBaryPointMax, "xkcd:red");
      // 	  // compPt->addLambda("chebichev_Min", computerChebichevMin, "xkcd:blue");
      // 	  // compPt->addLambda("chebichev_Max", computerChebichevMax, "xkcd:blue");
      // 	  // compPt->addLambda("comQP_min", computerCoMQPProjected, "xkcd:purple");
      // 	  compPt->addLambda("comQP", computerCoMQP, "xkcd:purple");
      // 	}
      // else
      // 	{
      // 	  // compPt->addLambda("baryPoint", computerBaryPoint, "xkcd:red");
      // 	  // compPt->addLambda("chebichev", computerChebichev, "xkcd:blue");
      // 	  compPt->addLambda("comQP", computerCoMQP, "xkcd:purple");
      // 	}
      compPt->addLambda("comQP", computerCoMQP, "xkcd:purple");
      
      compPt->addLambda("forceLF", forceLF, "xkcd:red");
      compPt->addLambda("forceRF", forceRF, "xkcd:blue");
      compPt->addLambda("forceRH", forceRH, "xkcd:green");
      
      computationPoints_.push_back(compPt);
    }

  std::cout << "Beginning computation of the equilibrium regions " << std::endl;
  float cpt(0), max(computationPoints_.size());
  
  auto start = std::chrono::high_resolution_clock::now();
  for (auto compPt : computationPoints_)
    {
      compPt->compute();
      cpt+=1.;
      std::cout << 100*cpt/max << " %\r";
      std::cout.flush();
    }
  
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop-start);
  std::cout << "Computation of the equilibrium regions done in "<< duration.count() << " ms" << std::endl;
  
  std::cout << "#-----------------------------" << std::endl;
  std::cout << std::endl;
}

void Experimenter::run_exp6()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running experiment for mode 6!" << std::endl;
  std::cout << std::endl;

  std::cout << "Loading files from folder: " << m_contactSetFileName << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  
  std::vector<std::string> contactSetNames;

  for (auto p: std::filesystem::directory_iterator(m_contactSetFileName))
    {
      contactSetNames.push_back(p.path().string());
    }

  auto grabNum = [](std::string file){
    int under = file.find("_")+1;
    int dot = file.find(".");
    return std::stoi(file.substr(under, dot-under));
  };

  auto compFiles = [grabNum](std::string file1, std::string file2){
    return grabNum(file1) < grabNum(file2);
  };
  
  std::sort(contactSetNames.begin(), contactSetNames.end(), compFiles);

  std::vector<std::shared_ptr<ContactSet>> contactSets;
  std::shared_ptr<ContactSet> contactSet;
  
  for (auto name: contactSetNames)
    {
      contactSet = std::make_shared<ContactSet>(!m_robust, name, m_numFrictionSides);
      contactSets.push_back(contactSet);
    }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop-start);
    
  std::cout << "Loaded "<< contactSets.size() <<  " contact Sets in " << duration.count() << "ms" << std::endl;

  std::cout << "Beginning computation of the equilibrium regions" << std::endl;
  int cpt(0), max(contactSets.size());

  for (auto contact: contactSets)
    {
      // auto contact = m_contactSets.at(0);
      std::cout << "Computing contactSet: " << contact->get_name() << std::endl;

      auto compute = [this](std::shared_ptr<RobustStabilityPolytope> polytope){
	// polytope->initSolver();
	polytope->projectionStabilityPolyhedron();
	// polytope->endSolver();
	// polytope->showPoly();
      };

      std::shared_ptr<RobustStabilityPolytope> polyNormal, polyThread;
      polyNormal = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, this->m_solver);
      polyThread = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, this->m_solver);

      polyThread->initSolver();
      std::thread myThread(compute, polyThread);
      myThread.join();
      polyThread->endSolver();

      polyNormal->initSolver();
      compute(polyNormal);
      polyNormal->endSolver();
      
      std::cout << "Computations done!" << std::endl;
      std::cout << "Let's compare the results:" << std::endl;
      polyNormal->showPoly();
      // polyThread->showPoly();
      // 0- checkbasic stuff
      if (polyNormal->get_numberOfVertices() != polyThread->get_numberOfVertices())
	{
	  std::cout << "Wrong number of vertices!" << std::endl;
	  throw 42;
	}
      // 1- check that the indexes are different for edges, vertices, faces
      auto vertices1 = polyNormal->vertices();
      auto vertices2 = polyThread->vertices();
      
      // 2- check the position of the points
      // 3- check if the structure is the same


      std::cout << "Everything seems fine\n" << std::endl;
    }

  // if no one failed maybe start again
  
  std::cout << "#-----------------------------" << std::endl;
  std::cout << std::endl;
}

// ---------- outputs and getters -----------
void Experimenter::save()
{

  // if the res folder doesn't exist, create it
  // std::cout << system("mkdir -p `rospack find stabiliplus`/res") << '\n';
  std::cout << system("mkdir -p /tmp/polytopes")<< " ";
  std::cout << system("mkdir -p /tmp/robots") << '\n';

  // creating new xml object
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLDeclaration * declaration = doc.NewDeclaration();
  doc.InsertEndChild(declaration);

  // creating root node
  tinyxml2::XMLNode * root = doc.NewElement("data");
  doc.InsertEndChild(root);

  // adding the mode to the root node
  tinyxml2::XMLElement * mode = doc.NewElement("mode");
  mode->SetAttribute("mode", m_mode);
  root->InsertEndChild(mode);

  // adding the robust bool to the root node to tell if we are using static or robust polytopes
  tinyxml2::XMLElement * robust = doc.NewElement("robust");
  robust->SetAttribute("robust", m_robust);
  root->InsertEndChild(robust);

  // adding the number of computed points (compPoint)
  tinyxml2::XMLElement * numComp = doc.NewElement("numComp");
  int numComputedPoints = computationPoints_.size();
  numComp->SetAttribute("numComputedPoints", numComputedPoints);
  root->InsertEndChild(numComp);
  
  int index(0);
  for (auto compPt : computationPoints_)
    {
      auto compPtXML = compPt->xmlComputationPoint(doc, index);
      root->InsertEndChild(compPtXML);
      index ++;
    }

  // save the xml to file
  std::string res_path= "/tmp/results.xml";
  // std::string res_path = stabiliplus_path+"/res/results.xml";
  doc.SaveFile(res_path.c_str());

  std::cout << "Experiment saved!" << '\n';
}
