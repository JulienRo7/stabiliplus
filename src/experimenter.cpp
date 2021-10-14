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
      polytope_ = std::make_shared<RobustStabilityPolytope> (contactSet_, maxIt_, precision_, solver_);
    }
  else
    {
      polytope_ = std::make_shared<StaticStabilityPolytope> (contactSet_, maxIt_, precision_, solver_);
    }

  polytope_->initSolver();

  try
  {
    polytope_->projectionStabilityPolyhedron();
  }
  catch (const std::runtime_error& e)
  {
  	std::cout << "Error code: " << polytope_->getErrorCode() << std::endl;
    std::cout << "Error: " << polytope_->getError() << " precision: " << polytope_->getMaxError() << std::endl;
    std::cout << "Iteration: " << polytope_->getIteration() << " maxIt: " << polytope_->getMaxIteration() << std::endl;
    // throw e;
  }
  // std::cout << "Error: " << polytope_->getError() << " precision: " << polytope_->getMaxError() << std::endl;
  // std::cout << "Iteration: " << polytope_->getIteration() << " maxIt: " << polytope_->getMaxIteration() << std::endl;

  auto stop = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop -start);

  polytope_->endSolver();
  
  totalTime_ = duration.count();

  // Compute the points that need to be saved using the lambda functions
  // computeOptimQP();
  // for (auto& cptPt: computerPoints_)
  //   {
  //     computedPoints_[cptPt.first] = cptPt.second(this);
  //   }
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
    case 2:
      run_exp2();
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
  compPt = std::make_shared<ComputationPoint>(m_contactSetFileName, m_numFrictionSides, m_solver, m_robust);

  computationPoints_.push_back(compPt);
  
  compPt->compute();
  // compPt->printTimings();    
  
}

void Experimenter::run_exp2()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running experiment for mode 2!" << std::endl;
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

  // creating the ComputationPoint obejcts
  std::shared_ptr<ComputationPoint> compPt;
  
  auto start = std::chrono::high_resolution_clock::now();
  for (auto name: contactSetNames)
    {
      compPt = std::make_shared<ComputationPoint> (name, m_numFrictionSides, m_solver, m_robust);   
      compPt->addLambda("baryPoint", computerBaryPoint, "xkcd:red");
      compPt->addLambda("chebichev", computerChebichev, "xkcd:blue");
      computationPoints_.push_back(compPt);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (end - start);
    std::cout << "ContactSets loaded in " << duration.count() << " ms" << std::endl;

  std::cout << "Beginning computation of the equilibrium regions " << std::endl;
  float cpt(0), max(computationPoints_.size());
  
  start = std::chrono::high_resolution_clock::now();
  for (auto compPt : computationPoints_)
    {
      compPt->compute();
      cpt+=1.;
      std::cout << 100*cpt/max << " %\r";
      std::cout.flush();
    }
  
  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds> (end-start);
  std::cout << "Computation of the equilibrium regions done in "<< duration.count() << " ms" << std::endl;
  
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
