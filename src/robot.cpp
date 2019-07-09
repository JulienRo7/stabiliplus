#include "robot.h"

using namespace std;

Robot::Robot() : m_gravity(0,0,-9.81), m_mass(1), m_numberOfFeet(4), m_numberOfFrictionSides(8),
m_numberOfAccelerations(0), m_numberOfIterations(0)
{
    m_accelerations.push_back(m_gravity);
    m_lp = glp_create_prob();
}

Robot::Robot(string const& robot_file_name, int numFrictionSides, int maxNumberOfIteration) :
m_gravity(0,0,-9.81), m_numberOfFrictionSides(numFrictionSides), m_maxNumberOfIteration(maxNumberOfIteration),
m_numberOfAccelerations(0), m_numberOfIterations(0),
m_lpMicro(0), m_innerConvexMicro(0), m_outerConvexMicro(0), m_supportFunctionMicro(0)
{
    loadRobot(robot_file_name);
    m_lp = glp_create_prob();
}

Robot::~Robot()
{
    // std::cout << "Robot destructor called!" << '\n';
    glp_delete_prob(m_lp);

    for (auto it: m_outerEdges)
    {
        it->finish();
    }

    // std::cout << "Old robot had faces: " << '\n';
    for (auto it: m_faces)
    {
        // std::cout << "Before finish() number of shared pointers for face " << it->get_index() << ": " << it.use_count() << '\n';
        // std::cout << it->get_index() << ", ";
        it->finish();
        // std::cout << "After finish() number of shared pointers for face " << it->get_index() << ": " << it.use_count() << '\n';
    }
    // std::cout << '\n';

}

void Robot::loadRobot(string const& file_name)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(file_name.c_str());

    if (doc.ErrorID() == 0)
    {
        tinyxml2::XMLElement* mainXML = doc.FirstChildElement("ROBOT");
        std::string mainType;
        tinyxml2::XMLElement* childXML(0);
        std::string childType;

        string robot_name;

        Eigen::Vector3d acceleration;
        tinyxml2::XMLElement* lineXML(0);


        while(mainXML)
        {
            mainType = mainXML->Value();
            if (std::strcmp(mainType.c_str(), "ROBOT")==0)
            {
                robot_name=mainXML->Attribute("name");
                childXML = mainXML->FirstChildElement();

                while (childXML) {
                    childType = childXML->Value();
                    if (std::strcmp(childType.c_str(), "Mass")==0)
                    {
                        childXML->QueryAttribute("mass", &m_mass);
                    }
                    else if (std::strcmp(childType.c_str(), "NumFeet")==0)
                    {
                        childXML->QueryAttribute("n_feet", &m_numberOfFeet);
                    }
                    else if (std::strcmp(childType.c_str(), "ContactPoint")==0)
                    {
                        m_feet.push_back(ContactPoints(childXML));
                        // cout << childXML->Attribute("name") << " added" << endl;
                    }
                    else {
                        std::cerr << "Un-Recognized child element name: " << childType << '\n';
                    }
                    childXML = childXML->NextSiblingElement();
                }

            }
            else if (std::strcmp(mainType.c_str(), "ACCELERATIONS")==0)
            {
                childXML = mainXML->FirstChildElement();

                while (childXML)
                {
                    ++m_numberOfAccelerations;
                    // std::cout << "Add Acceleration!" << '\n';
                    childType = childXML->Value();
                    if (std::strcmp(childType.c_str(), "matrix")==0)
                    {
                        lineXML = childXML->FirstChildElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(0));
                        lineXML = lineXML->NextSiblingElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(1));
                        lineXML = lineXML->NextSiblingElement("line");
                        lineXML->FirstChildElement("v")->QueryDoubleText(&acceleration(2));

                        // std::cout << "New acceleration: " << '\n' << acceleration << '\n';

                        m_accelerations.push_back(acceleration);

                    }
                    else
                    {
                        std::cerr << "Un-Recognized child element name: " << childType << '\n';
                    }

                    childXML = childXML->NextSiblingElement();
                }
            }
            else
            {
                std::cerr << "Un-Recognized main element name: " << mainType << '\n';
            }

            mainXML = mainXML->NextSiblingElement();
        }

        // cout << robot_name << " loaded !" << endl;
    }
    else
    {
        cerr << "Failed to open the XML description file" << endl;
    }
}

void Robot::showRobot()
{
    cout << "Mass of the robot: " << m_mass << endl;
    cout << "Number of feet of the robot: " << m_numberOfFeet << endl;
}

Eigen::MatrixXd Robot::computeMatrixA1()
{
    int n_columns = 3*m_numberOfFeet;

    Eigen::MatrixXd A1(6,n_columns);

    for (int i(0); i<m_numberOfFeet; ++i)
    {
        A1.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
        A1.block<3,3>(3,3*i) = skewSymmetric(m_feet[i].get_position());
    }

    return A1;
}

Eigen::MatrixXd Robot::computeMatrixA2(Eigen::Vector3d const& acceleration)
{
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6,3);
    A2.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
    A2.block<3,3>(3,0) = -skewSymmetric(acceleration);

    return A2;
}

Eigen::VectorXd Robot::computeVector_t(Eigen::Vector3d const& acceleration)
{
    Eigen::VectorXd t(6);

    t.head(3) = -m_mass*acceleration;
    t.tail(3) = Eigen::Vector3d::Zero();

    return t;
}

Eigen::MatrixXd Robot::buildMatrixA()
{
    int const n_columnsA1 = 3*m_numberOfFeet;
    int const n_columnsA = 3*m_numberOfFeet*m_numberOfAccelerations + 3;
    int const n_rowsA = 6*m_numberOfAccelerations;

    Eigen::MatrixXd  A = Eigen::MatrixXd::Zero(n_rowsA, n_columnsA);
    Eigen::MatrixXd A1(6, n_columnsA1);

    A1 = computeMatrixA1();

    for (int i=0; i<m_numberOfAccelerations; ++i)
    {
        A.block(6*i, n_columnsA1*i, 6, n_columnsA1) = A1;
        A.block<6, 3>(6*i, n_columnsA-3) = computeMatrixA2(m_accelerations[i]);
    }

    return A;

}

Eigen::VectorXd Robot::buildVectorB()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(6*m_numberOfAccelerations);

    for (int i=0; i < m_numberOfAccelerations; ++i)
    {
        B.segment<6>(6*i) = computeVector_t(m_accelerations[i]);
    }
    return B;
}

Eigen::MatrixXd Robot::buildFrictionF()
{
    int const numberOfColumns = 3*m_numberOfFeet*m_numberOfAccelerations + 3;
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2)*m_numberOfAccelerations + 6;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(numberOfRows, numberOfColumns);
    Eigen::MatrixXd F_contact((m_numberOfFrictionSides+2), 3);

    for (int i=0; i<m_numberOfFeet; ++i)
    {
        F_contact = m_feet[i].linearizedFrictionCone(m_numberOfFrictionSides);

        for (int j=0; j<m_numberOfAccelerations; ++j)
        {
            F.block(j*(m_numberOfFrictionSides+2)*m_numberOfFeet + i*(m_numberOfFrictionSides+2), j*3*m_numberOfFeet + i*3 ,(m_numberOfFrictionSides+2), 3) = F_contact;
        }
    }

    F.block<3,3>(numberOfRows-6,numberOfColumns-3) = Eigen::Matrix3d::Identity();
    F.bottomRightCorner<3,3>() = -Eigen::Matrix3d::Identity();

    return F;
}

Eigen::VectorXd Robot::buildFrictionVectorf()
{
    int const numberOfRows = m_numberOfFeet*(m_numberOfFrictionSides+2)*m_numberOfAccelerations + 6;
    Eigen::VectorXd f(numberOfRows);

    double f_max = 10*m_mass;

    for (int i=0; i<numberOfRows-6; i+=m_numberOfFrictionSides+2)
    {
        f[i] = f_max;
        f[i+1] = 0;
        for (int j=0; j<m_numberOfFrictionSides; ++j)
        {
            f[i+2+j]= 0.0;
        }
    }
    for (int i=numberOfRows-6; i<numberOfRows; i++)
    {
        f[i] = 1; // CoM position limited to the unit cube
    }

    return f;
}

void Robot::buildStabilityProblem()
{


    Eigen::MatrixXd A;
    A = buildMatrixA();
    // std::cout << "A:" << '\n' << A << '\n';

    Eigen::VectorXd B = buildVectorB();
    // std::cout << "b:" << '\n' << B << '\n';

    Eigen::MatrixXd F;
    F = buildFrictionF();
    // std::cout << "F:" << '\n' << F << '\n';

    Eigen::VectorXd f;
    f = buildFrictionVectorf();
    // std::cout << "f:" << '\n' << f << '\n';

    int numberOfColumns = A.cols();
    int numberOfRows = A.rows()+F.rows();

    // std::cout << "Number of Columns: " << numberOfColumns << '\n';
    // std::cout << "Number of Rows: " << numberOfRows << '\n';


    double const fmax(10*m_mass);


    glp_set_obj_dir(m_lp, GLP_MAX); // The objective here is to maximize
    //
    glp_add_rows(m_lp, numberOfRows);

    for (int i=0; i<B.size(); ++i)
    {
        glp_set_row_bnds(m_lp, i+1, GLP_FX, B(i), B(i));
    }

    for (int i=B.size(); i<numberOfRows; i++)
    {
        glp_set_row_bnds(m_lp, i+1, GLP_UP, 0.0, f[i-B.size()]);
    }

    glp_add_cols(m_lp, numberOfColumns);
    for (int i = 0; i<numberOfColumns; ++i)
    {
        glp_set_col_bnds(m_lp, i+1, GLP_FR, -100.0, 100.0);
        glp_set_obj_coef(m_lp, i+1, 0.0);
    }

    glp_set_col_bnds(m_lp, numberOfColumns-2, GLP_DB, -10.0, 10.0);
    glp_set_col_bnds(m_lp, numberOfColumns-1, GLP_DB, -10.0, 10.0);
    glp_set_col_bnds(m_lp, numberOfColumns-0, GLP_DB, -1.0, 1.0);

    // // ia[1] = 1, ja[1] = 1, ar[1] = 1.0;

    int ia[1+(numberOfRows-6)*numberOfColumns], ja[1+(numberOfRows-6)*numberOfColumns];
    double ar[1+(numberOfRows-6)*numberOfColumns];

    for (int i=0; i<A.rows(); ++i)
    {
        for (int j=0; j<numberOfColumns; ++j)
        {
            ia[1+i*numberOfColumns + j]=1+i;
            ja[1+i*numberOfColumns + j]=1+j;
            ar[1+i*numberOfColumns + j]=A(i,j);
            // std::cout << 1+i*numberOfColumns + j << " " << 1+i <<" " << 1+j <<" " << A(i,j) << '\n';
        }
    }
    for (int i=A.rows(); i<numberOfRows-6; ++i)
    {
        for (int j=0; j<numberOfColumns; ++j)
        {
            ia[1+i*numberOfColumns + j]=1+i;
            ja[1+i*numberOfColumns + j]=1+j;
            ar[1+i*numberOfColumns + j]=F(i-A.rows(),j);
            // ar[1+i*numberOfColumns + j]=0;
            // std::cout << 1+i*numberOfColumns + j << " " << 1+i <<" " << 1+j <<" " << F(i-6*m_numberOfAccelerations,j) << '\n';
        }
    }

    glp_load_matrix(m_lp, (numberOfRows-6)*numberOfColumns, ia, ja, ar);

    glp_term_out(GLP_OFF);
}

void Robot::buildReducedStabilityProblem()
{

    Eigen::VectorXd B = buildVectorB();

    Eigen::MatrixXd A = buildMatrixA();
    // std::cout << "A: "<< '\n' << A << '\n';
    Eigen::MatrixXd F = buildFrictionF();
    // std::cout << "F: "<< '\n' << F << '\n';

    Eigen::VectorXd f = buildFrictionVectorf();

    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A.transpose());
    // Eigen::MatrixXd P = qr.colsPermutation();
    auto start = std::chrono::high_resolution_clock::now();
    const Eigen::MatrixXd& Q = qr.householderQ();
    m_Q_c = Q.leftCols(A.rows());//.setLength(qr.nonzeroPivots());
    m_Q_u = Q.rightCols(A.cols()-A.rows());//.setLength(qr.nonzeroPivots());
    m_R_inv_T_b = qr.matrixQR().topRows(A.rows()).transpose().triangularView<Eigen::Lower>().solve(B);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    // std::cout << "QR decomposition time: " << duration.count() << " microseconds"<< '\n';

    // std::cout << "P matrix of A: " << '\n' << P << '\n';
    // std::cout << "Qc matrix of A: " << '\n' << Q_c << '\n';
    // std::cout << "Q matrix of A: " << '\n' << Q << '\n';
    // std::cout << "R matrix of A: " << '\n' << R << '\n';
    // std::cout << "Inverse of R: " << '\n' << R_inv << '\n';

    Eigen::MatrixXd F_bis = F*m_Q_u;

    Eigen::VectorXd f_bis = f - (F*m_Q_c)*m_R_inv_T_b;

    // std::cout << "F_bis: " << '\n' << F_bis << '\n';
    // std::cout << "f_bis: " << '\n' << f_bis << '\n';

    int const numberOfColumns = F_bis.cols();
    int const numberOfRows = F_bis.rows();


    glp_set_obj_dir(m_lp, GLP_MAX); // The objective here is to maximize
    //
    glp_add_rows(m_lp, numberOfRows);


    for (int i = 0; i<numberOfRows; i++)
    {
        glp_set_row_bnds(m_lp, i+1, GLP_UP, 0.0, f_bis[i]);
    }


    glp_add_cols(m_lp, numberOfColumns);
    for (int i = 0; i<numberOfColumns; ++i)
    {
        glp_set_col_bnds(m_lp, i+1, GLP_FR, -100.0, 100.0);
        // glp_set_obj_coef(m_lp, i+1, 0.0);
    }

    int ia[1+numberOfRows*numberOfColumns], ja[1+numberOfRows*numberOfColumns];
    double ar[1+numberOfRows*numberOfColumns];

    for (int i=0; i<numberOfRows; ++i)
    {
        for (int j=0; j<numberOfColumns; ++j)
        {
            ia[1+i*numberOfColumns + j]=1+i;
            ja[1+i*numberOfColumns + j]=1+j;
            ar[1+i*numberOfColumns + j]=F_bis(i,j);
            // std::cout << 1+i*numberOfColumns + j << " " << 1+i <<" " << 1+j <<" " << A(i,j) << '\n';
        }
    }

    // std::cout << ia << '\n';
    glp_load_matrix(m_lp, numberOfRows*numberOfColumns, ia, ja, ar);

    glp_term_out(GLP_OFF);
    // std::cout << "Reduced Stability Problem built!" << '\n';
    // glp_write_lp(m_lp, NULL, "export_lp.txt");
}

void Robot::solveStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point)
{
    int numberOfColumns = glp_get_num_cols(m_lp);

    glp_set_obj_coef(m_lp, numberOfColumns-2, direction(0));
    glp_set_obj_coef(m_lp, numberOfColumns-1, direction(1));
    glp_set_obj_coef(m_lp, numberOfColumns-0, direction(2));



    glp_simplex(m_lp, NULL);


    point << glp_get_col_prim(m_lp, numberOfColumns-2),
              glp_get_col_prim(m_lp, numberOfColumns-1),
              glp_get_col_prim(m_lp, numberOfColumns-0);
    // std::cout << "Objective Function : " << glp_get_obj_val(m_lp) << '\n';

}

void Robot::solveReducedStabilityProblem(Eigen::Vector3d const& direction, Eigen::Vector3d &point)
{
    Eigen::VectorXd c = Eigen::VectorXd::Zero(3*m_numberOfFeet*m_numberOfAccelerations+3);
    c.tail(3)=direction;

    const Eigen::VectorXd c_bis = m_Q_u.transpose()*c;

    for (int i = 0; i<c_bis.size(); ++i)
    {
        glp_set_obj_coef(m_lp, i+1, c_bis[i]);
    }
    glp_simplex(m_lp, NULL);

    Eigen::VectorXd z(c_bis.size());

    for (int i = 0; i<c_bis.size(); ++i)
    {
        z[i]=glp_get_col_prim(m_lp, i+1);
    }

    const Eigen::VectorXd x = m_Q_c*m_R_inv_T_b + (m_Q_u*z);

    // std::cout << "m_Q_c*m_R_inv_T_b=" << '\n' << (m_Q_c*m_R_inv_T_b).transpose() << '\n';

    point = x.tail(3);
}

void Robot::projectionStabilityPolyhedron()
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
        // solveReducedStabilityProblem(initialDirections[i], point);
        auto start = std::chrono::high_resolution_clock::now();
        solveReducedStabilityProblem(initialDirections[i], point);
        // solveStabilityProblem(initialDirections[i], point);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_lpMicro += duration.count();

        newVertex = std::make_shared<Vertex> (point, initialDirections[i]);
        m_vertices.push_back(newVertex);
        // std::cout << "Search direction: " << initialDirections[i].transpose() << '\n';
        // std::cout << "New Vertex coordinates :" << newVertex->get_coordinates().transpose() << '\n';

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

    // start = std::chrono::high_resolution_clock::now();
    // computeSupportFunctions();
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    // m_supportFunctionMicro+=duration.count();

    // std::shared_ptr<Face> dirFace;

    while (m_numberOfIterations < m_maxNumberOfIteration)
    {

        m_numberOfIterations++;
        // std::cout << "----- Iteration number: " << m_numberOfIterations << " -----" << '\n';
        auto dirFace = *max_element(m_faces.begin(), m_faces.end(), Face::compareFacesMeasure);


        auto start = std::chrono::high_resolution_clock::now();
        solveReducedStabilityProblem(dirFace->get_normal(), point);
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
        computeSupportFunctions(dirFace);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_supportFunctionMicro+=duration.count();
    }
}

Eigen::Vector3d Robot::computeInnerPoint()
{
    // Computation of the inner point: it is used to make sure that the normal of the faces are oriented toward the outside
    m_innerPoint = Eigen::Vector3d::Zero();
    for (auto it : m_vertices)
    {
        m_innerPoint += it->get_coordinates();
    }
    m_innerPoint /= m_vertices.size();
}

void Robot::buildInnerPoly()
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

void Robot::updateInnerPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace)
{
    std::list<std::shared_ptr<Face>> consideredFaces;
    std::vector<std::shared_ptr<Face>> currentNeighbors;


    consideredFaces.push_back(dirFace);
    // dirFace = nullptr;
    // currentNeighbors = dirFace->findNeighbors();
    // consideredFaces.splice(consideredFaces.end(), currentNeighbors);

    std::vector<std::shared_ptr<Face>> visibleFaces;
    std::vector<std::shared_ptr<Edge>> visibleEdges;

    auto currentFace = consideredFaces.begin();

    // std::cout << "New Vertex coordiantes :" << newVertex->get_coordinates().transpose() << '\n';
    while (currentFace!=consideredFaces.end())
    {
        if (!(*currentFace)->pointInHalfSpace(newVertex->get_coordinates()))
        {
            visibleFaces.push_back(*currentFace);
            // std::cout << "Face " << (*currentFace)->get_index() << " is visible!" << '\n';
            // std::cout << "Face normal : " << (*currentFace)->get_normal().transpose() << " and offset " << (*currentFace)->get_offset() << '\n';

            auto currentEdges = (*currentFace)->get_edges();
            for (auto it_e : currentEdges)
            {
                if (find(visibleEdges.begin(), visibleEdges.end(), it_e)==visibleEdges.end())
                {
                    visibleEdges.push_back(it_e);
                }
            }

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

    // consideredFaces.erase(consideredFaces.begin(), consideredFaces.end());

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

void Robot::buildOuterPoly()
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

void Robot::updateOuterPoly(std::shared_ptr<Vertex> &newVertex, std::shared_ptr<Face> &dirFace)
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

void Robot::computeSupportFunctions(std::shared_ptr<Face>& dirFace)
{
    // std::cout << "shared pointer on dirFace use count: " << dirFace.use_count() << '\n';
    // std::cout << "Current Support Function: " << '\n';
    // for (auto it: m_faces)
    // {
    //     std::cout << it->get_supportFunction() << ", ";
    // }
    // std::cout << '\n';

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

bool Robot::computeSupportFunction(std::shared_ptr<Face>& face, const std::shared_ptr<OuterVertex>& initPoint)
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
        return true;
    }
}


// ----------- output and display functions ----------
void Robot::exportVertices()
{

    ofstream file_stream("vertices.txt");

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
                        << it_vertices->get_direction()[2] << ';' << endl;
        }

        for (auto it : m_edges)
        {
            file_stream << "ie;" // ie = inner edge
                        << it->get_vertex1()->get_coordinates()[0] << ';'
                        << it->get_vertex1()->get_coordinates()[1] << ';'
                        << it->get_vertex1()->get_coordinates()[2] << ';'
                        << it->get_vertex2()->get_coordinates()[0] << ';'
                        << it->get_vertex2()->get_coordinates()[1] << ';'
                        << it->get_vertex2()->get_coordinates()[2] << ';' << endl;
        }

        for (auto const& it : m_outerVertices)
        {
            file_stream << "ov;" // oe = outer vertex
                        << it->get_coordinates()[0] << ';'
                        << it->get_coordinates()[1] << ';'
                        << it->get_coordinates()[2] << ';' << endl;
        }

        for (auto const& it : m_outerEdges)
        {
            file_stream << "oe;" // oe = outer edge
                        << it->get_outerVertex1()->get_coordinates()[0] << ';'
                        << it->get_outerVertex1()->get_coordinates()[1] << ';'
                        << it->get_outerVertex1()->get_coordinates()[2] << ';'
                        << it->get_outerVertex2()->get_coordinates()[0] << ';'
                        << it->get_outerVertex2()->get_coordinates()[1] << ';'
                        << it->get_outerVertex2()->get_coordinates()[2] << ';' << endl;
        }
    }
    else
    {
        std::cerr << "Error: Impossible to open the output file." << '\n';
    }

    // glp_write_lp(m_lp, NULL, "export_lp.txt");
}

void Robot::showPoly()
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

// ------------------ Getters -----------------------

int Robot::get_numberOfFeet() const
{
    return m_numberOfFeet;
}

int Robot::get_numberOfAcceletations() const
{
    return m_numberOfAccelerations;
}

int Robot::get_numberOfVertices() const
{
    return m_vertices.size();
}

int Robot::get_numberOfFaces() const
{
    return m_faces.size();
}

int Robot::get_numberOfOuterVertices() const
{
    return m_outerVertices.size();
}

int Robot::get_numberOfOuterFaces() const
{
    return m_outerFaces.size();
}

double Robot::get_lpMicro() const
{
    return m_lpMicro;
}

double Robot::get_innerConvexMicro() const
{
    return m_innerConvexMicro;
}

double Robot::get_outerConvexMicro() const
{
    return m_outerConvexMicro;
}

double Robot::get_supportFunctionMicro() const
{
    return m_supportFunctionMicro;
}

// ------------------ setter -----------------------
void Robot::set_maxNumberOfIterations(int maxNumberOfIteration)
{
    m_maxNumberOfIteration = maxNumberOfIteration;
}

// ---------- Static function -----------

Eigen::Matrix3d Robot::skewSymmetric(Eigen::Vector3d const& vect)
{
    Eigen::Matrix3d vect_hat;
    vect_hat << 0, -vect(2), vect(1),
                vect(2), 0, -vect(0),
                -vect(1), vect(0), 0;
    return vect_hat;
}
