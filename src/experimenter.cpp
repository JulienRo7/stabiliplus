#include "stabiliplus/experimenter.h"


// ---------- constructors and destructor -----------

Experimenter::Experimenter(int mode, std::string const& robot_file_name, int numFrictionSides):
    m_mode(mode),
    m_robot(robot_file_name, numFrictionSides)
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
    switch (m_mode) {
        case 1:
            run_exp1();
            break;
        case 2:
            run_exp2();
            break;
        case 3:
            run_exp3();
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
    std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(m_robot,50));

    polytope->buildStabilityProblem();
    polytope->projectionStabilityPolyhedron();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    m_polytopes.push_back(polytope);

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
    std::cout << "Well... it has not been implemented yet!" << '\n';
}

void Experimenter::run_exp3()
{
    std::cout << "Running Experiment for mode 3!" << '\n';
    if (m_robot.get_name()=="robot_8")
    {
        Eigen::Vector3d dx;
        dx << 0.0,
              0.01,
              0.0;

        for (int i=0; i<50; i++)
        {
            std::shared_ptr<StabilityPolytope> polytope(new StabilityPolytope(m_robot));
            polytope->buildStabilityProblem();
            polytope->projectionStabilityPolyhedron();
            m_polytopes.push_back(polytope);

            m_robot.translateContact(3, dx);
        }
    }
    else
    {
        std::cerr << "This experiment requires robot 8 to be loaded!" << '\n';
    }
}


// ---------- outputs and getters -----------
void Experimenter::save()
{

    // if the res folder doesn't exist, create it
    // std::cout << system("mkdir -p `rospack find stabiliplus`/res") << '\n';
    std::cout << system("mkdir -p /tmp/polytopes") << '\n';
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
        poly->get_robot()->saveRobot(robot_file_name);

        tinyxml2::XMLElement *compPoint = doc.NewElement("compPoint");
        compPoint->SetAttribute("index", poly_count);
        root->InsertEndChild(compPoint);

        tinyxml2::XMLElement *polyXML = doc.NewElement("poly");
        polyXML->SetAttribute("file_name", poly_file_name.c_str());
        compPoint->InsertEndChild(polyXML);

        tinyxml2::XMLElement *robotXML = doc.NewElement("robot");
        robotXML->SetAttribute("file_name", robot_file_name.c_str());
        compPoint->InsertEndChild(robotXML);


        poly_count++;
    }


    // save the xml to file
    std::string res_path = "/tmp/results.xml";
    // std::string res_path = stabiliplus_path+"/res/results.xml";
    doc.SaveFile( res_path.c_str() );



    std::cout << "Experiment saved!" << '\n';
}
