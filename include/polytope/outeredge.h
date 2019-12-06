#ifndef OUTEREDGE_H_INCLUDED
#define OUTEREDGE_H_INCLUDED

#include "outerface.h"
#include "outervertex.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

// class Edge; // Forward declaration

class OuterEdge : public std::enable_shared_from_this<OuterEdge>
{
public:
  // ---------- constructors ----------
  OuterEdge(std::shared_ptr<OuterVertex> outerVertex1, std::shared_ptr<OuterVertex> outerVertex2);

  // ---------- destructor ----------
  ~OuterEdge();

  // ---------- other functions ----------
  void init();
  void finish();

  // ---------- getters ----------
  int get_index() const;
  std::shared_ptr<OuterVertex> get_outerVertex1() const;
  std::shared_ptr<OuterVertex> get_outerVertex2() const;
  std::shared_ptr<OuterVertex> get_otherOuterVertex(std::shared_ptr<OuterVertex> outerVertex) const;

  std::shared_ptr<OuterFace> get_outerFace1() const;
  std::shared_ptr<OuterFace> get_outerFace2() const;
  void add_outerFacesToVector(std::vector<std::shared_ptr<OuterFace>> & vectorOuterFaces);

  // ---------- setters ----------
  void set_outerVertice1(std::shared_ptr<OuterVertex> outerVertex);
  void set_outerVertice2(std::shared_ptr<OuterVertex> outerVertex);
  void switch_outerVertices(std::shared_ptr<OuterVertex> oldVertex, std::shared_ptr<OuterVertex> newVertex);

private:
  static int GlobalOuterEdgeCounter;
  int m_index;

  std::shared_ptr<OuterVertex> m_outerVertex1;
  std::shared_ptr<OuterVertex> m_outerVertex2;

  std::shared_ptr<OuterFace> m_outerFace1;
  std::shared_ptr<OuterFace> m_outerFace2;
};

#endif // OUTEREDGE_H_INCLUDED
