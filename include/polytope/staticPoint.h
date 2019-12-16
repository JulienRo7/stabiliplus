#ifndef STATIC_POINT_H_INCLUDE
#define STATIC_POINT_H_INCLUDE

// standard libraries
#include <fstream>
#include <iostream>
#include <memory>

// libraries
#include <Eigen/Dense>

enum StaticMeasure
{
  AREA,
  SUPPORT
};

class StaticPoint
{
public:
  // ----- constructors and destructors -----
  StaticPoint(Eigen::Vector2d dir, Eigen::Vector2d vertex, StaticMeasure mesType = AREA);
  ~StaticPoint();

  // ----- class's main methods -----
  void updateNormal();
  void updateOuterVertex();
  void updateMeasure();
  void update();

  void computeArea();
  void computeSupport();
  // ----- outputs -----
  void writeToStream(std::ofstream & file_stream) const;
  Eigen::Vector4d plane() const;

  // ----- getters -----
  std::shared_ptr<StaticPoint> next() const;
  std::shared_ptr<StaticPoint> prec() const;

  
  Eigen::Vector2d innerVertex() const;
  inline int size() const
  {
    return static_cast<int>(innerVertex().size());
  }
  inline double & x() const
  {
    return innerVertex().x(); 
  }
  inline double & y() const
  {
    return innerVertex().y(); 
  }

  // -- easy access for the vertex 

  Eigen::Vector2d searchDir() const;
  Eigen::Vector2d normal() const;
  Eigen::Vector2d outerVertex() const;
  double measure() const;

  // ----- setters -----
  void next(std::shared_ptr<StaticPoint> next_pt);
  void prec(std::shared_ptr<StaticPoint> prec_pt);

  // ----- operator overload -----
  bool operator()(const std::shared_ptr<StaticPoint> p1, const std::shared_ptr<StaticPoint> p2);

private:
  Eigen::Vector2d m_innerVertex;
  Eigen::Vector2d m_searchDir;
  Eigen::Vector2d m_normal;
  Eigen::Vector2d m_outerVertex;

  std::shared_ptr<StaticPoint> m_next;
  std::shared_ptr<StaticPoint> m_prec;

  StaticMeasure m_measureType;
  double m_measure;
};

#endif // STATIC_POINT_H_INCLUDE
