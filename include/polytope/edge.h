#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED

#include "vertex.h"
#include <memory>

// #include "face.h"
class Face; // Forward declaration

class Edge
{
public:
  Edge(std::shared_ptr<Vertex> vertex1, std::shared_ptr<Vertex> vertex2);

  ~Edge();

  bool addFace(const std::shared_ptr<Face> & face);
  void removeFace(const std::shared_ptr<Face> & face);

  bool isSame(const Edge & b) const;

  // ----------- getters ----------
  int get_index() const;
  std::shared_ptr<Face> get_otherFace(std::shared_ptr<Face> face) const;

  std::shared_ptr<Vertex> get_vertex1() const;
  std::shared_ptr<Vertex> get_vertex2() const;

  std::shared_ptr<Face> get_face1() const;
  std::shared_ptr<Face> get_face2() const;

private:
  static int GlobalEdgeCounter;
  int m_index;

  std::shared_ptr<Vertex> m_vertex1;
  std::shared_ptr<Vertex> m_vertex2;

  std::shared_ptr<Face> m_face1;
  std::shared_ptr<Face> m_face2;
};

bool operator==(Edge const & a, Edge const & b);

#endif // EDGE_H_INCLUDED
