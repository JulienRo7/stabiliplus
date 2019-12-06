#include "polytope/outeredge.h"

int OuterEdge::GlobalOuterEdgeCounter = 0;

// ---------- constructors ----------
OuterEdge::OuterEdge(std::shared_ptr<OuterVertex> outerVertex1, std::shared_ptr<OuterVertex> outerVertex2)
: m_index(GlobalOuterEdgeCounter), m_outerVertex1(outerVertex1), m_outerVertex2(outerVertex2)
{
  ++GlobalOuterEdgeCounter;

  auto faces1 = m_outerVertex1->get_outerFaces();
  auto faces2 = m_outerVertex2->get_outerFaces();

  std::vector<std::shared_ptr<OuterFace>> outerFaces;

  std::sort(faces1.begin(), faces1.end());
  std::sort(faces2.begin(), faces2.end());

  std::set_intersection(faces1.begin(), faces1.end(), faces2.begin(), faces2.end(), back_inserter(outerFaces));

  if(outerFaces.size() == 2)
  {
    m_outerFace1 = outerFaces[0];
    m_outerFace2 = outerFaces[1];
  }
  else
  {
    std::cerr << "Error: there are too many faces or not enough!" << '\n';
  }
}

// ---------- destructor ----------
OuterEdge::~OuterEdge()
{
  // finish();
}

// ---------- other functions ----------
void OuterEdge::init()
{
  m_outerVertex1->add_outerEdge(shared_from_this());
  m_outerVertex2->add_outerEdge(shared_from_this());
}

void OuterEdge::finish()
{
  m_outerVertex1->remove_outerEdge(shared_from_this());
  m_outerVertex2->remove_outerEdge(shared_from_this());
}

// ---------- getters ----------
int OuterEdge::get_index() const
{
  return m_index;
}

std::shared_ptr<OuterVertex> OuterEdge::get_outerVertex1() const
{
  return m_outerVertex1;
}

std::shared_ptr<OuterVertex> OuterEdge::get_outerVertex2() const
{
  return m_outerVertex2;
}

std::shared_ptr<OuterVertex> OuterEdge::get_otherOuterVertex(std::shared_ptr<OuterVertex> OuterVertex) const
{
  if(OuterVertex == m_outerVertex1)
  {
    return m_outerVertex2;
  }
  else
  {
    return m_outerVertex1;
  }
}

std::shared_ptr<OuterFace> OuterEdge::get_outerFace1() const
{
  return m_outerFace1;
}

std::shared_ptr<OuterFace> OuterEdge::get_outerFace2() const
{
  return m_outerFace2;
}

void OuterEdge::add_outerFacesToVector(std::vector<std::shared_ptr<OuterFace>> & vectorOuterFaces)
{

  if(find(vectorOuterFaces.begin(), vectorOuterFaces.end(), m_outerFace1) == vectorOuterFaces.end())
  {
    vectorOuterFaces.push_back(m_outerFace1);
  }

  if(find(vectorOuterFaces.begin(), vectorOuterFaces.end(), m_outerFace2) == vectorOuterFaces.end())
  {
    vectorOuterFaces.push_back(m_outerFace2);
  }
}

// ---------- setters ----------
void OuterEdge::set_outerVertice1(std::shared_ptr<OuterVertex> outerVertex)
{
  m_outerVertex1->remove_outerEdge(shared_from_this());
  m_outerVertex1 = outerVertex;
  m_outerVertex1->add_outerEdge(shared_from_this());
}
void OuterEdge::set_outerVertice2(std::shared_ptr<OuterVertex> outerVertex)
{
  m_outerVertex2->remove_outerEdge(shared_from_this());
  m_outerVertex2 = outerVertex;
  m_outerVertex2->add_outerEdge(shared_from_this());
}

void OuterEdge::switch_outerVertices(std::shared_ptr<OuterVertex> oldVertex, std::shared_ptr<OuterVertex> newVertex)
{
  if(oldVertex == m_outerVertex1)
  {
    set_outerVertice1(newVertex);
  }
  else if(oldVertex == m_outerVertex2)
  {
    set_outerVertice2(newVertex);
  }
  else
  {
    std::cerr << "The outer edge does not have this outer vertex" << '\n';
  }
}
