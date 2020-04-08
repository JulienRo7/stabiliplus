#include "experimenter.h"

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
void Experimenter::computePoint(std::shared_ptr<ContactSet> contactSet)
{
  std::shared_ptr<StabilityPolytope> polytope;
  auto start = std::chrono::high_resolution_clock::now();
  if (m_robust)
    {
      if (contactSet->hasConstrainedContact())
	{
	  polytope = std::make_shared<ConstrainedEquilibriumPolytope> (contactSet, 50, 0.05, m_solver);
	}
      else
	{
	  polytope = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, m_solver);
	}
    }
  else
    {
      polytope = std::make_shared<StaticStabilityPolytope> (contactSet, 50, 0.01, m_solver);
    }

  polytope->initSolver();
  polytope->projectionStabilityPolyhedron();
  
  auto stop = std::chrono::high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop -start);
  
  m_polytopes.push_back(polytope);
  m_total_times.push_back(duration.count());
}

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
    case 3:
      run_exp3();
      break;
    case 4:
      run_exp4();
      break;
    case 5:
      run_exp5();
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
  
  std::shared_ptr<ContactSet> contactSet;
  contactSet = std::make_shared<ContactSet>(!m_robust, m_contactSetFileName, m_numFrictionSides);
  contactSet->showContactSet();
  m_contactSets.push_back(contactSet);

  computePoint(contactSet);
  
  auto polytope = m_polytopes[0];
  
  if (m_robust and !contactSet->hasConstrainedContact())
    {
      std::cout << "Computation time: " << m_total_times[0] << "ms for " << polytope->get_numberOfVertices() << " inner Vertices"
		<< " and " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_numberOfOuterVertices() << " outer vertices." << '\n'
		<< "Number of Inner Faces: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_numberOfFaces()
		<< ", number of outer faces: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_numberOfOuterFaces() << '\n'
	<< "LP time: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).LPTime() << " µs" << '\n'
	<< "inner time: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_innerConvexMicro() << " µs" << '\n'
	<< "outer time: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_outerConvexMicro() << " µs" << '\n'
	<< "support time: " << dynamic_cast<RobustStabilityPolytope&>(*polytope).get_supportFunctionMicro() << " µs" << std::endl;
    }
  else
    {
      std::cout << "Equilibrium  Region computed in " << m_total_times[0] << " ms for " << polytope->get_numberOfVertices()
		<< " inner Vertices" << std::endl;
      std::cout << "Init time: " << polytope->initTime() << " µs" << std::endl;
      std::cout << "LP time: " << polytope->LPTime() << " µs with solver " << m_solver << std::endl;
      std::cout << "Structure time: " << polytope->structTime() << " µs" << std::endl;
    }
}

void Experimenter::run_exp2()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running Experiment for mode 2!" << '\n';

  std::string robot_names[4] = {"./robots/robot_1.xml", "./robots/robot_2.xml", "./robots/robot_3.xml",
                                "./robots/robot_4.xml"};
  int numTrials = 100;
  //Solver solvers[3] = {GLPK, LP_SOLVE, GUROBI};
  Solver solvers[1] = {GLPK};

  bool staticCase = !m_robust;

  for(auto solver : solvers)
  {
    for(auto rob_file : robot_names)
    {
      std::shared_ptr<ContactSet> rob = std::make_shared<ContactSet>(staticCase, rob_file, m_numFrictionSides);
      for(int i = 0; i < numTrials; i++)
      {
        std::cout << "Solver: " << solver << " ContactSet: " << rob_file << " Run: " << i + 1 << '/' << numTrials
                  << '\n';
        auto start = std::chrono::high_resolution_clock::now();

	std::shared_ptr<StabilityPolytope> polytope;
	if(m_robust)
	  {
	    polytope = std::make_shared<RobustStabilityPolytope> (rob, 50, 0.05, solver);
	  }
	else
	  {
	    polytope = std::make_shared<StaticStabilityPolytope> (rob, 50, 0.01, solver);
	  }
        

        polytope->initSolver();
        polytope->projectionStabilityPolyhedron();

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	m_contactSets.push_back(rob);
        m_polytopes.push_back(polytope);
        m_total_times.push_back(duration.count());
      }
    }
  }  
}

void Experimenter::run_exp3()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running Experiment for mode 3!" << '\n';

  std::shared_ptr<ContactSet> contactSet;
  std::shared_ptr<StabilityPolytope> polytope;

  Eigen::Vector3d dx;
  dx << 0.0, 0.01, 0.0;
  
  // if(contactSet->get_name() == "robot_8")
  // {
  for(int i = 0; i < 50; i++)
    {
      contactSet = std::make_shared<ContactSet>(!m_robust, m_contactSetFileName, m_numFrictionSides);
      contactSet->translateContact(3, i*dx);
      
      auto start = std::chrono::high_resolution_clock::now();
      
      if(m_robust)
	{
	  polytope = std::make_shared<RobustStabilityPolytope> (contactSet, 50, 0.05, m_solver);
	}
      else
	{
	  polytope = std::make_shared<StaticStabilityPolytope> (contactSet, 50, 0.01, m_solver);
	}

      polytope->initSolver();
      polytope->projectionStabilityPolyhedron();

      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

      m_contactSets.push_back(contactSet);
      m_polytopes.push_back(polytope);
      m_total_times.push_back(duration.count());
    }
  // }
  // else
  // {
  //   std::cerr << "This experiment requires robot 8 to be loaded!" << '\n';
  // }
  
}

void Experimenter::run_exp4()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running experiment for mode 4!" << std::endl;

  std::shared_ptr<ContactSet> contactSet;
  
  std::string contactName("contact_exp4");
  
  Eigen::Matrix4d homTrans;
  homTrans << 1, 0, 0, 1,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  double fmax(10);
  double df = fmax/20;
  int n_pts(60);

  auto f = [&] (int i) {
    if (i <= 20)
      {
	return 0.;
      }
    else
      {
	if (i>=40)
	  {
	    return fmax;
	  }
	else
	  {
	    return (i-20)*df;
	  }
      }
  };

  
  for (int i=0; i<= n_pts; i++)
    {
      // Load the contact set
      contactSet = std::make_shared<ContactSet> (!m_robust, m_contactSetFileName, m_numFrictionSides);
  
      // Add a contact point with fmax to 0 
      contactSet->addContact(contactName, homTrans, 0.5, f(i), 0);

      computePoint(contactSet);
      
      m_contactSets.push_back(contactSet);
    }
}


void Experimenter::run_exp5()
{
  std::cout << "#-----------------------------" << std::endl;
  std::cout << "Running experiment for mode 5!" << std::endl;
  std::cout << std::endl;

  std::cout << "Loading files from folder: " << m_contactSetFileName << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  
  std::vector<std::string> contactSetNames;
  std::string contactSetName;

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


  std::shared_ptr<ContactSet> contactSet;
  
  for (auto name: contactSetNames)
    {
      contactSet = std::make_shared<ContactSet>(!m_robust, name, m_numFrictionSides);
      m_contactSets.push_back(contactSet);
    }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop-start);
    
  std::cout << "Loaded "<< m_contactSets.size() <<  " contact Sets in " << duration.count() << "ms" << std::endl;

  std::cout << "Beginning computation of the equilibrium regions" << std::endl;
  float cpt(0), max(m_contactSets.size());
  
  start = std::chrono::high_resolution_clock::now();
  for (auto contactSet: m_contactSets)
    {
      std::cout << contactSet->get_name() << std::endl;
      computePoint(contactSet);
      cpt+=1.;
      // std::cout << 100*cpt/max << " %\r";
      // std::cout.flush();
    }
  stop = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop-start);
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
  int numComputedPoints = m_polytopes.size();
  numComp->SetAttribute("numComputedPoints", numComputedPoints);
  root->InsertEndChild(numComp);

  // adding the polytopes that were created
  std::string poly_file_name, robot_file_name;
  int poly_count = 0;

  for(auto poly : m_polytopes)
  {
    // poly_file_name = stabiliplus_path+"/res/polytope_"+std::to_string(poly_count)+".txt";
    // robot_file_name = stabiliplus_path+"/res/robot_"+std::to_string(poly_count)+".xml";
    poly_file_name = "/tmp/polytopes/polytope_" + std::to_string(poly_count) + ".txt";
    robot_file_name = "/tmp/robots/robot_" + std::to_string(poly_count) + ".xml";

    std::ofstream stream(poly_file_name);
    poly->writeToStream(stream);

    if(typeid(ContactSet) == typeid(*(poly->problemDescriptor())))
    {
      auto csPtr = static_cast<ContactSet *>(poly->problemDescriptor().get());
      csPtr->saveContactSet(robot_file_name);
    }

    tinyxml2::XMLElement * compPoint = doc.NewElement("compPoint");
    compPoint->SetAttribute("index", poly_count);
    root->InsertEndChild(compPoint);

    tinyxml2::XMLElement * polyXML = doc.NewElement("poly");
    polyXML->SetAttribute("file_name", poly_file_name.c_str());
    compPoint->InsertEndChild(polyXML);

    tinyxml2::XMLElement * robotXML = doc.NewElement("robot");

    if(typeid(ContactSet) == typeid(*(poly->problemDescriptor())))
    {

      auto csPtr = static_cast<ContactSet *>(poly->problemDescriptor().get());
      csPtr->saveContactSet(robot_file_name);

      robotXML->SetAttribute("name", csPtr->get_name().c_str());
    }

    robotXML->SetAttribute("file_name", robot_file_name.c_str());
    compPoint->InsertEndChild(robotXML);

    tinyxml2::XMLElement * timeXML = doc.NewElement("times");
    timeXML->SetAttribute("total", m_total_times.at(poly_count));
    timeXML->SetAttribute("LP", poly->LPTime());
    timeXML->SetAttribute("init", poly->initTime());
    timeXML->SetAttribute("struct", poly->structTime());
    // timeXML->SetAttribute("inner", poly->innerConvexMicro());
    // timeXML->SetAttribute("outer", poly->outerConvexMicro());
    // timeXML->SetAttribute("support", poly->supportFunctionMicro());
    compPoint->InsertEndChild(timeXML);

    tinyxml2::XMLElement * solverXML = doc.NewElement("solver");
    switch(poly->solverType())
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
    compPoint->InsertEndChild(solverXML);

    poly_count++;
  }

  // save the xml to file
  std::string res_path = "/tmp/results.xml";
  // std::string res_path = stabiliplus_path+"/res/results.xml";
  doc.SaveFile(res_path.c_str());

  std::cout << "Experiment saved!" << '\n';
}
