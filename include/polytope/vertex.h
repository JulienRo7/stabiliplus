#ifndef VERTEX_H_INCLUDED
#define VERTEX_H_INCLUDED

#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <vector>

class Vertex
{
public:
  Vertex(Eigen::Vector3d coordinates, Eigen::Vector3d direction);
  ~Vertex();

  void addInnerNeighbor(std::shared_ptr<Vertex> const neigh);
  void removeInnerNeighbor(std::shared_ptr<Vertex> const neigh);

  bool isSame(Vertex const & b) const;

  // ----- Getters -----
  int get_index() const;
  Eigen::Vector3d get_coordinates() const;
  Eigen::Vector3d get_direction() const;
  double get_offset() const;

private:
  static int GlobalVertexCounter;
  int m_index;

  Eigen::Vector3d m_coordinates;
  Eigen::Vector3d m_direction;
  double m_offset;

  // inner polyhedron related stuff
  std::vector<std::shared_ptr<Vertex>> m_innerNeighbors;
};

bool operator==(Vertex const & a, Vertex const & b);

#endif // VERTEX_H_INCLUDED
