#include "robot.h"

using namespace std;

Robot::Robot() : m_gravity(0,0,-9.81), m_mass(1), m_numberOfFeet(4), m_numberOfFrictionSides(8),
m_numberOfAccelerations(0), m_numberOfIterations(0)
{
    m_accelerations.push_back(m_gravity);
    m_lp = glp_create_prob();
}

Robot::Robot(string const& robot_file_name, int numFrictionSides) : m_gravity(0,0,-9.81),
m_numberOfFrictionSides(numFrictionSides), m_numberOfAccelerations(0), m_numberOfIterations(0)
{
    loadRobot(robot_file_name);
    m_lp = glp_create_prob();
}

Robot::~Robot()
{
    glp_delete_prob(m_lp);

    for (std::vector<Face*>::iterator it = m_faces.begin(); it!=m_faces.end(); it++)
    {
        delete *it;
        *it = 0;
    }

    for (std::vector<Edge*>::iterator it=  m_edges.begin();it!=m_edges.end(); it++)
    {
        delete *it;
        *it = 0;
    }

    for (std::vector<Vertex*>::iterator it=m_vertices.begin(); it!=m_vertices.end(); it++)
    {
        delete *it;
        *it = 0;
    }


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
    Eigen::MatrixXd A2(6,3);
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

    Eigen::MatrixXd  A(n_rowsA, n_columnsA);
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
    Eigen::VectorXd B(6*m_numberOfAccelerations);

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

    std::cout << "QR decomposition time: " << duration.count() << " microseconds"<< '\n';

    // std::cout << "P matrix of A: " << '\n' << P << '\n';
    // std::cout << "Qc matrix of A: " << '\n' << Q_c << '\n';
    // std::cout << "Qu matrix of A: " << '\n' << m_Q_u << '\n';
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
    std::array<Eigen::Vector3d, 4> initialDirections;
    Eigen::Vector3d dir;
    Eigen::Vector3d point;

    Vertex* newVertex(0);
    Face* dirFace(0);

    double lpMilis  = 0;
    double convexMilis =0;

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
        lpMilis += duration.count();

        newVertex = new Vertex(point, initialDirections[i]);
        m_vertices.push_back(newVertex);
    }

    buildInnerPoly();

    while (m_numberOfIterations < 50)
    {
        m_numberOfIterations++;

        dirFace = *max_element(m_faces.begin(), m_faces.end(), Face::compareFacesArea);

        auto start = std::chrono::high_resolution_clock::now();
        solveReducedStabilityProblem(dirFace->get_normal(), point);
        // solveStabilityProblem(dirFace->get_normal(), point);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        lpMilis += duration.count();

        const double check = dirFace->get_normal().dot(point) - dirFace->get_offset();
        // std::cout << "Check: "<< check << '\n';
        if (check > 0.001)
        {
            newVertex = new Vertex(point, dirFace->get_normal());
            m_vertices.push_back(newVertex);

            dirFace = *max_element(m_faces.begin(), m_faces.end(), Face::compareFacesArea);
            // std::cout << "Next research face: " << dirFace->get_index() << '\n';
            auto start = std::chrono::high_resolution_clock::now();
            updateInnerPoly(newVertex, dirFace);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            convexMilis+=duration.count();

        }
        else
        {
            dirFace->set_area_null();
        }


    }
    std::cout << "Total LP time: " << lpMilis << " microseconds" << '\n';
    std::cout << "Total convex time: " << convexMilis << " microseconds" << '\n';
}

Eigen::Vector3d Robot::computeInnerPoint()
{
    // Computation of the inner point: it is used to make sure that the normal of the faces are oriented toward the outside
    m_innerPoint = Eigen::Vector3d::Zero();
    for (std::vector<Vertex*>::iterator it=m_vertices.begin(); it != m_vertices.end(); it++)
    {
        m_innerPoint += (*it)->get_coordinates();
    }
    m_innerPoint /= m_vertices.size();
}

void Robot::buildInnerPoly()
{
    computeInnerPoint();
    // std::cout << "Inner Point: " << m_innerPoint << '\n';

    Edge* newEdge(0);

    newEdge = new Edge(m_vertices[0], m_vertices[1]);
    m_edges.push_back(newEdge);

    newEdge = new Edge(m_vertices[0], m_vertices[2]);
    m_edges.push_back(newEdge);

    newEdge = new Edge(m_vertices[0], m_vertices[3]);
    m_edges.push_back(newEdge);

    newEdge = new Edge(m_vertices[1], m_vertices[2]);
    m_edges.push_back(newEdge);

    newEdge = new Edge(m_vertices[1], m_vertices[3]);
    m_edges.push_back(newEdge);

    newEdge = new Edge(m_vertices[2], m_vertices[3]);
    m_edges.push_back(newEdge);

    Face* newFace(0);
    newFace = new Face(m_vertices[0], m_vertices[1], m_vertices[2], m_edges[0], m_edges[1], m_edges[3], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = new Face(m_vertices[0], m_vertices[1], m_vertices[3], m_edges[0], m_edges[2], m_edges[4], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = new Face(m_vertices[0], m_vertices[2], m_vertices[3], m_edges[1], m_edges[2], m_edges[5], m_innerPoint);
    m_faces.push_back(newFace);
    newFace = new Face(m_vertices[1], m_vertices[2], m_vertices[3], m_edges[3], m_edges[4], m_edges[5], m_innerPoint);
    m_faces.push_back(newFace);



}

void Robot::updateInnerPoly(Vertex* newVertex, Face* dirFace)
{
    std::list<Face*> consideredFaces;
    std::list<Face*> currentNeighbors;
    std::list<Edge*> currentEdges;

    consideredFaces.push_back(dirFace);

    // currentNeighbors = dirFace->findNeighbors();
    // consideredFaces.splice(consideredFaces.end(), currentNeighbors);

    std::list<Face*> visibleFaces;
    std::list<Edge*> visibleEdges;

    std::list<Face*>::iterator currentFace = consideredFaces.begin();

    while (currentFace!=consideredFaces.end())
    {
        // std::cout << "Current face: " << (*currentFace)->get_index() << '\n';
        if (!(*currentFace)->pointInHalfSpace(newVertex->get_coordinates()))
        {
            // std::cout << "Visible face!" << '\n';
            visibleFaces.push_back(*currentFace);

            currentEdges = (*currentFace)->get_edges();
            unionEdgeLists(visibleEdges, currentEdges);

            currentNeighbors = (*currentFace)->findNeighbors();
            unionFaceLists(consideredFaces, currentNeighbors);
        }

        currentFace++;

    }

    // the discrimination of the edges that should be remove could be done in the previous loop

    std::list<Edge*> edges_to_keep;
    std::list<Edge*> edges_to_delete;

    for (std::list<Edge*>::iterator it=visibleEdges.begin(); it!=visibleEdges.end(); it++)
    {
        if (find(visibleFaces.begin(), visibleFaces.end(), (*it)->get_face1())!=visibleFaces.end() and
            find(visibleFaces.begin(), visibleFaces.end(), (*it)->get_face2())!=visibleFaces.end())
        {
            // std::cout << "Edge to delete: " << *it << '\n';
            edges_to_delete.push_back(*it);
        }
        else
        {
            // std::cout << "Edge to keep: " << *it << '\n';
            edges_to_keep.push_back(*it);
        }
    }


    //remove old faces
    std::vector<Face*>::iterator posFace;
    for (std::list<Face*>::iterator it = visibleFaces.begin(); it!=visibleFaces.end(); it++)
    {
        posFace = find(m_faces.begin(), m_faces.end(), *it);
        if (posFace!= m_faces.end())
        {
            // std::cout << "Visible face found: " << (*it)->get_index() << '\n';
            delete *posFace;
            m_faces.erase(posFace);
            *it = 0;
        }
        else
        {
            // std::cout << "Visible face not found!" << '\n';
        }

    }

    // remove old edges:
    std::vector<Edge*>::iterator posEdge;
    for (std::list<Edge*>::iterator it = edges_to_delete.begin(); it!=edges_to_delete.end(); it++)
    {
        posEdge = find(m_edges.begin(), m_edges.end(), *it);
        if (posEdge!= m_edges.end())
        {
            // std::cout << "Visible edge found: " << (*it)->get_index() << '\n';
            delete *posEdge;
            m_edges.erase(posEdge);
            *it = 0;
        }
        else
        {
            // std::cout << "Visible edge not found!" << '\n';
        }

    }

    Face* newFace(0);
    Edge* newEdge1(0);
    Edge* newEdge2(0);

    std::map<Vertex*, Edge*> processedVertex;

    for (std::list<Edge*>::iterator it = edges_to_keep.begin(); it!=edges_to_keep.end(); it++)
    {
        if (processedVertex.find((*it)->get_vertex1())==processedVertex.end())
        {
            newEdge1 = new Edge(newVertex, (*it)->get_vertex1());
            m_edges.push_back(newEdge1);
            processedVertex[(*it)->get_vertex1()] = newEdge1;
        }
        else
        {
            newEdge1 = processedVertex.find((*it)->get_vertex1())->second;
        }

        if (processedVertex.find((*it)->get_vertex2())==processedVertex.end())
        {
            newEdge2 = new Edge(newVertex, (*it)->get_vertex2());
            m_edges.push_back(newEdge2);
            processedVertex[(*it)->get_vertex2()] = newEdge2;
        }
        else
        {
            newEdge2 = processedVertex.find((*it)->get_vertex2())->second;
        }

        // std::cout << "Old edge: "<< (*it)->get_index()<<", new edge 1: " << newEdge1->get_index() << ", new edge 2: "<< newEdge2->get_index() << '\n';
        // showPoly();
        newFace = new Face(newVertex, (*it)->get_vertex1(), (*it)->get_vertex2(), *it, newEdge1, newEdge2, m_innerPoint);
        m_faces.push_back(newFace);
    }

    // showPoly();

}

void Robot::buildOuterPoly()
{
    // Vertex* outerVertex(0);
    // Eigen::Vector3d outerPoint;
    // Eigen::Vector3d b;
    // Eigen::Matrix3d A;
    //
    //
    // for (auto it = m_faces.begin(); it!=m_faces.end(); it++)
    // {
    //     A << (*it)->get_vertex1()->get_direction().transpose(),
    //       << (*it)->get_vertex2()->get_direction().transpose(),
    //       << (*it)->get_vertex3()->get_direction().transpose();
    //
    //     b << (*it)->get_vertex1()->get_offset(),
    //       << (*it)->get_vertex2()->get_offset(),
    //       << (*it)->get_vertex3()->get_offset(),
    //
    //     outerPoint = A.solve(b);
    }
}

// ----------- output and display functions ----------
void Robot::exportVertices()
{

    ofstream file_stream("vertices.txt");

    if (file_stream)
    {

        for (std::vector<Vertex*>::iterator it_vertices = m_vertices.begin(); it_vertices!=m_vertices.end(); ++it_vertices)
        {
            file_stream << "v;"
                        << (*it_vertices)->get_coordinates()[0] << ';'
                        << (*it_vertices)->get_coordinates()[1] << ';'
                        << (*it_vertices)->get_coordinates()[2] << ';'
                        << (*it_vertices)->get_direction()[0] << ';'
                        << (*it_vertices)->get_direction()[1] << ';'
                        << (*it_vertices)->get_direction()[2] << ';' << endl;
        }

        for (std::vector<Edge*>::iterator it = m_edges.begin(); it!=m_edges.end(); it++)
        {
            file_stream << "e;"
                        << (*it)->get_vertex1()->get_coordinates()[0] << ';'
                        << (*it)->get_vertex1()->get_coordinates()[1] << ';'
                        << (*it)->get_vertex1()->get_coordinates()[2] << ';'
                        << (*it)->get_vertex2()->get_coordinates()[0] << ';'
                        << (*it)->get_vertex2()->get_coordinates()[1] << ';'
                        << (*it)->get_vertex2()->get_coordinates()[2] << ';' << endl;
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
    for (std::vector<Vertex*>::iterator it = m_vertices.begin(); it!=m_vertices.end()-1; it++)
    {
        std::cout << (*it)->get_index() << ", ";
    }
    std::cout << m_vertices.back()->get_index() << '\n';

    std::cout << "Current Edges: " << '\n';
    for (std::vector<Edge*>::iterator it = m_edges.begin(); it!=m_edges.end(); it++)
    {
        std::cout << "Edge: " << (*it)->get_index() << " has vertex " << (*it)->get_vertex1()->get_index()
                  << " and "<< (*it)->get_vertex2()->get_index() << '\n';
    }

    std::cout << '\n';
}

// ------------------ Getters -----------------------

int Robot::get_numberOfFeet()
{
    return m_numberOfFeet;
}

int Robot::get_numberOfAcceletations()
{
    return m_numberOfAccelerations;
}

int Robot::get_numberOfVertices()
{
    return m_vertices.size();
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

void Robot::unionEdgeLists(std::list<Edge*>& listA, std::list<Edge*>& listB)
{
    for (std::list<Edge*>::iterator it=listB.begin(); it != listB.end(); it++)
    {
        if (find(listA.begin(), listA.end(), *it)==listA.end())
        {
            listA.push_back(*it);
        }
    }
}

void Robot::unionFaceLists(std::list<Face*>& listA, std::list<Face*>& listB)
{
    for (std::list<Face*>::iterator it=listB.begin(); it != listB.end(); it++)
    {
        if (find(listA.begin(), listA.end(), *it)==listA.end())
        {
            listA.push_back(*it);
        }
    }
}
