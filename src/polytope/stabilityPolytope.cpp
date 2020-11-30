#include "polytope/stabilityPolytope.h"

StabilityPolytope::StabilityPolytope(std::shared_ptr<ProblemDescriptor> inputPD,
                                     int maxIteration,
                                     double maxError,
                                     Solver solverType)
: m_pdPtr(inputPD), m_solverType(solverType), m_iteration(0), m_maxIteration(maxIteration), m_error(1000),
  m_maxError(maxError), m_LPTime(0), m_initTime(0), m_structTime(0), m_solverEnded(false)
{
  switch(m_solverType)
  {
    case GLPK:
      m_lp = new GlpkWrapper;
      break;

      /*
    case LP_SOLVE:
      m_lp = new LPSolveWrapper;
      break;

    case GUROBI:
      m_lp = new GurobiWrapper;
      break;
      */
  }
}

StabilityPolytope::~StabilityPolytope()
{
  endSolver();
  // delete m_pdPtr;
  // m_pdPtr = nullptr;
}

// ----------- main class methods ----------

void StabilityPolytope::endSolver()
{
  if (!m_solverEnded)
    {
      delete m_lp;
      m_lp = nullptr;
    }
  m_solverEnded = true;
}


bool StabilityPolytope::stopCriterion() const // return true when the algorithm must stop
{
  return (m_iteration >= m_maxIteration) || (m_error <= m_maxError);
}

// ----------- output and display functions ----------

// simple implementation of the function in order to not implemented for every subclass
tinyxml2::XMLElement * StabilityPolytope::xmlPolytope(tinyxml2::XMLDocument & doc) const
{
  auto xmlPoly = doc.NewElement("polytope");
  xmlPoly->SetAttribute("type", "stability");
  return xmlPoly; 
}

void StabilityPolytope::saveToFile(std::string fileName) const
{
  tinyxml2::XMLDocument doc;
  
  auto declaration = doc.NewDeclaration();
  doc.InsertEndChild(declaration);
  
  doc.InsertEndChild(xmlPolytope(doc));
  
  doc.SaveFile(fileName.c_str());
  // std::cout << "[StabilityPolytope::saveToFile] I reach here 3!" << std::endl;
}

// ----------- getters ----------

double StabilityPolytope::LPTime() const
{
  return m_LPTime;
}

double StabilityPolytope::initTime() const
{
  return m_initTime;
}

double StabilityPolytope::structTime() const
{
  return m_structTime;
}

Solver StabilityPolytope::solverType() const
{
  return m_solverType;
}

Eigen::Vector3d StabilityPolytope::chebichevCenter() const
{
  return Eigen::Vector3d::Zero();
}
// ------------------ setter -----------------------

void StabilityPolytope::maxIteration(int maxIteration)
{
  m_maxIteration = maxIteration;
}

// ------------------ static functions ? -----------------------
Eigen::Vector3d StabilityPolytope::chebichevCenter(std::vector<Eigen::Vector4d> planes) const
{
  glp_prob * m_lp;
  m_lp = glp_create_prob();

  glp_set_obj_dir(m_lp, GLP_MAX);

  glp_add_cols(m_lp, 4);
  glp_set_col_bnds(m_lp, 1, GLP_FR, 0.0, 0.0);
  glp_set_col_bnds(m_lp, 2, GLP_FR, 0.0, 0.0);
  glp_set_col_bnds(m_lp, 3, GLP_FR, 0.0, 0.0);
  glp_set_col_bnds(m_lp, 4, GLP_LO, 0.0, 0.0); // R>=0

  int n = planes.size();
  glp_add_rows(m_lp, n);
  
  for (int i=0; i<n; i++)
  {
    glp_set_row_bnds(m_lp, i + 1, GLP_UP, 0.0, planes[i](3));
  }

  int ia[1 + 4*n], ja[1 + 4*n];
  double ar[1 + 4*n];

  auto matCoef = [planes](int i, int j){
    if (j<3)
      {
	return planes[i](j);
      }
    else
      {
	double a, b, c, d;
	a = planes[i](0);
	b = planes[i](1);
	c = planes[i](2);
	d = std::hypot(a, b, c);
	return d;
      }
  };
  
  for (int i=0; i<n; i++)
    {
      for (int j=0; j<4; j++)
	{
	  ia[1 + 4*i + j]= 1 + i;
	  ja[1 + 4*i + j]= 1 + j;
	  ar[1 + 4*i + j]= matCoef(i, j);
	}
    }
  glp_load_matrix(m_lp, 4*n, ia, ja, ar);
  glp_term_out(GLP_OFF);

  glp_set_obj_coef(m_lp, 1, 0);
  glp_set_obj_coef(m_lp, 2, 0);
  glp_set_obj_coef(m_lp, 3, 0);
  glp_set_obj_coef(m_lp, 4, 1);

  //glp_write_lp(m_lp, NULL, "/tmp/export_lp.txt");
  
  int out = glp_simplex(m_lp, NULL);
  //glp_simplex(m_lp, NULL);

  Eigen::Vector3d chebichevCenter;
  double radius;

  chebichevCenter << glp_get_col_prim(m_lp, 1),
    glp_get_col_prim(m_lp, 2),
    glp_get_col_prim(m_lp, 3);

  radius = glp_get_col_prim(m_lp, 4);
  
  glp_delete_prob(m_lp);

  //std::cout << "GLPK output: " << out << std::endl;
  //std::cout << "Found Chebichev Center: " << chebichevCenter.transpose() << " with radius: " << radius << std::endl;

  return chebichevCenter;
}
