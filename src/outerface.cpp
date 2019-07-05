#include "outerface.h"

// #include "outervertex.h"

int OuterFace::GlobalOuterFaceCounter = 0;

// ---------- constructors ----------

OuterFace::OuterFace(Vertex* innerVertex):
    m_index(GlobalOuterFaceCounter),
    m_innerVertex(innerVertex)
{
    ++GlobalOuterFaceCounter;
}

// ---------- destructor ----------

OuterFace::~OuterFace()
{

}

// ---------- class functions ----------
// bool OuterFace::isFaceOf(OuterVertex* outerVertex)
// {
//     auto vertexOuterFaces = outerVertex->get_outerFaces();
//     if (find(vertexOuterFaces.begin(), vertexOuterFaces.end(), this)!=vertexOuterFaces.end())
//     {
//         return TRUE;
//     }
//     else
//     {
//         return FALSE;
//     }
// }

// ---------- getters ----------

int OuterFace::get_index() const
{
    return m_index;
}

Vertex* OuterFace::get_innerVertex() const
{
    return m_innerVertex;
}

Eigen::Vector3d OuterFace::get_normal() const
{
    return m_innerVertex->get_direction();
}

double OuterFace::get_offset() const
{
    return m_innerVertex->get_offset();
}
