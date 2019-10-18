#include "stabiliplus/experimenter.h"


// ---------- constructors and destructor -----------

Experimenter::Experimenter(int mode, std::string const& contact_set_file_name, int numFrictionSides, Solver solver):
  m_mode(mode), m_numFrictionSides(numFrictionSides),
  m_contactSet(contact_set_file_name, numFrictionSides), m_solver(solver)
{
    stabiliplus_path = "";
}

Experimenter::~Experimenter()
{
}


// ---------- main functions -----------
void Experimenter::run()
{
    std::cout << "Running experiment!" << '\n';
    switch (m_mode)
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
      default:
	std::cerr << "Unknown mode" << '\n';
	
    }
    std::cout << "Experiment done!" << '\n';
}

void Experimenter::run_exp1()
{
    std::cout << "Running Experiment for mode 1!" << '\n';

    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(m_contactSet, 50, m_solver));

    polytope->buildStabilityProblem();
    polytope->projectionStabilityPolyhedron();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    m_polytopes.push_back(polytope);
    m_total_times_ms.push_back(duration.count());

    std::cout << "Computation time: " << duration.count() << "ms for " << polytope->get_numberOfVertices() << " inner Vertices"
              << " and " << polytope->get_numberOfOuterVertices() << " outer vertices."<< '\n';
    std::cout << "Number of Inner Faces: " << polytope->get_numberOfFaces() << ", number of outer faces: " << polytope->get_numberOfOuterFaces() << '\n';

    std::cout << "LP time: " << polytope->get_lpMicro() << " microseconds" << '\n';
    std::cout << "inner time: " << polytope->get_innerConvexMicro() << " microseconds" << '\n';
    std::cout << "outer time: " << polytope->get_outerConvexMicro() << " microseconds" << '\n';
    std::cout << "support time: " << polytope->get_supportFunctionMicro() << " microseconds" << '\n';
}

void Experimenter::run_exp2()
{
    std::cout << "Running Experiment for mode 2!" << '\n';

    std::string robot_names[4] = {"./robots/robot_1.xml", "./robots/robot_2.xml", "./robots/robot_3.xml", "./robots/robot_4.xml"};
    int numTrials = 100;
    Solver solvers[3] = {GLPK, LP_SOLVE, GUROBI};

    
    for (auto solver: solvers)
      {
	for (auto rob_file: robot_names)
	  {
	    ContactSet rob(rob_file, m_numFrictionSides);
	    for (int i = 0; i<numTrials; i++)
	      {
		std::cout << "Solver: " << solver << " ContactSet: " << rob_file << " Run: " << i+1 << '/' << numTrials << '\n';
		auto start = std::chrono::high_resolution_clock::now();

		std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(rob, 50, solver));
		polytope->buildStabilityProblem();
		polytope->projectionStabilityPolyhedron();

		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

		m_polytopes.push_back(polytope);
		m_total_times_ms.push_back(duration.count());
	      }
	  }
      }
    
}

void Experimenter::run_exp3()
{
    std::cout << "Running Experiment for mode 3!" << '\n';
    if (m_contactSet.get_name()=="robot_8")
    {
        Eigen::Vector3d dx;
        dx << 0.0,
              0.01,
              0.0;

        for (int i=0; i<50; i++)
        {
	  auto start = std::chrono::high_resolution_clock::now();

	  std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(m_contactSet,50, m_solver));
	  polytope->buildStabilityProblem();
	  polytope->projectionStabilityPolyhedron();

	  auto stop = std::chrono::high_resolution_clock::now();
	  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	  m_polytopes.push_back(polytope);
	  m_total_times_ms.push_back(duration.count());

	  m_contactSet.translateContact(3, dx);
        }
    }
    else
    {
        std::cerr << "This experiment requires robot 8 to be loaded!" << '\n';
    }
}

void Experimenter::run_exp4()
{
  std::cout << "Welcome to mode 4: static stability" << std::endl;

  auto start = std::chrono::high_resolution_clock::now();
  
  StaticStabilityPolytope static_poly(m_contactSet, 50, 0.01, m_solver);
  
  static_poly.initSolver();
  static_poly.projectionStabilityPolyhedron();

  auto stop = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds> (stop - start);
  double totalTime  = duration.count();

  std::cout << "Static Stability Region computed in " << totalTime << " microseconds." << std::endl;
  std::cout << "Init time: " << static_poly.initTime() << " microseconds" << std::endl;
  std::cout << "LP time: " << static_poly.LPTime() << " microseconds with solver " << m_solver << std::endl;
  std::cout << "Structure time: " << static_poly.structTime() << " microseconds" << std::endl;
  static_poly.saveResults("/tmp/static_res.txt");
}


// ---------- outputs and getters -----------
void Experimenter::save()
{

    // if the res folder doesn't exist, create it
    // std::cout << system("mkdir -p `rospack find stabiliplus`/res") << '\n';
    std::cout << system("mkdir -p /tmp/polytopes");
    std::cout << system("mkdir -p /tmp/robots") << '\n';

    // creating new xml object
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLDeclaration * declaration = doc.NewDeclaration();
    doc.InsertEndChild(declaration);

    // creating root node
    tinyxml2::XMLNode *root = doc.NewElement("data");
    doc.InsertEndChild(root);

    // adding the mode to the root node
    tinyxml2::XMLElement *mode = doc.NewElement("mode");
    mode->SetAttribute("mode", m_mode);
    root->InsertEndChild(mode);

    // adding the number of computed points (compPoint)
    tinyxml2::XMLElement *numComp = doc.NewElement("numComp");
    int numComputedPoints = m_polytopes.size();
    numComp->SetAttribute("numComputedPoints", numComputedPoints);
    root->InsertEndChild(numComp);

    // adding the polytopes that were created
    std::string poly_file_name, robot_file_name;
    int poly_count = 0;

    for (auto poly: m_polytopes)
    {
        // poly_file_name = stabiliplus_path+"/res/polytope_"+std::to_string(poly_count)+".txt";
        // robot_file_name = stabiliplus_path+"/res/robot_"+std::to_string(poly_count)+".xml";
        poly_file_name = "/tmp/polytopes/polytope_"+std::to_string(poly_count)+".txt";
        robot_file_name = "/tmp/robots/robot_"+std::to_string(poly_count)+".xml";
        poly->exportVertices(poly_file_name);
        poly->get_contactSet()->saveContactSet(robot_file_name);

        tinyxml2::XMLElement *compPoint = doc.NewElement("compPoint");
        compPoint->SetAttribute("index", poly_count);
        root->InsertEndChild(compPoint);

        tinyxml2::XMLElement *polyXML = doc.NewElement("poly");
        polyXML->SetAttribute("file_name", poly_file_name.c_str());
        compPoint->InsertEndChild(polyXML);

        tinyxml2::XMLElement *robotXML = doc.NewElement("robot");
	robotXML->SetAttribute("name", poly->get_contactSet()->get_name().c_str());
        robotXML->SetAttribute("file_name", robot_file_name.c_str());
        compPoint->InsertEndChild(robotXML);

	tinyxml2::XMLElement *timeXML = doc.NewElement("times");
	timeXML->SetAttribute("total", m_total_times_ms.at(poly_count));
	timeXML->SetAttribute("LP", poly->get_lpMicro());
	timeXML->SetAttribute("inner", poly->get_innerConvexMicro());
	timeXML->SetAttribute("outer", poly->get_outerConvexMicro());
	timeXML->SetAttribute("support", poly->get_supportFunctionMicro());
	compPoint->InsertEndChild(timeXML);

	tinyxml2::XMLElement *solverXML = doc.NewElement("solver");
	switch(poly->get_solver())
	  {
	  case GLPK:
	    solverXML->SetAttribute("name","GLPK");
	    break;

	  case LP_SOLVE:
	    solverXML->SetAttribute("name","LP_SOLVE");
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
    doc.SaveFile( res_path.c_str() );



    std::cout << "Experiment saved!" << '\n';
}
