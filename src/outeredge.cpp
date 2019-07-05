#include "outeredge.h"

int OuterEdge::GlobalOuterEdgeCounter = 0;

// ---------- constructors ----------
OuterEdge::OuterEdge(OuterVertex* outerVertex1, OuterVertex* outerVertex2):
    m_index(GlobalOuterEdgeCounter),
    m_outerVertex1(outerVertex1), m_outerVertex2(outerVertex2)
{
    ++GlobalOuterEdgeCounter;

    m_outerVertex1->add_outerEdge(this);
    m_outerVertex2->add_outerEdge(this);

    auto faces1 = m_outerVertex1->get_outerFaces();
    auto faces2 = m_outerVertex2->get_outerFaces();

    std::vector<OuterFace*> outerFaces;

    std::sort(faces1.begin(), faces1.end());
    std::sort(faces2.begin(), faces2.end());

    std::set_intersection(faces1.begin(), faces1.end(),
                          faces2.begin(), faces2.end(),
                          back_inserter(outerFaces));

    if (outerFaces.size()==2)
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
    m_outerVertex1->remove_outerEdge(this);
    m_outerVertex2->remove_outerEdge(this);
}


// ---------- getters ----------
int OuterEdge::get_index() const
{
    return m_index;
}

OuterVertex* OuterEdge::get_outerVertex1() const
{
    return m_outerVertex1;
}

OuterVertex* OuterEdge::get_outerVertex2() const
{
    return m_outerVertex2;
}

OuterVertex* OuterEdge::get_otherOuterVertex(OuterVertex* OuterVertex) const
{
    if (OuterVertex == m_outerVertex1)
    {
        return m_outerVertex2;
    }
    else
    {
        return m_outerVertex1;
    }
}

OuterFace* OuterEdge::get_outerFace1() const
{
    return m_outerFace1;
}

OuterFace* OuterEdge::get_outerFace2() const
{
    return m_outerFace2;
}


void OuterEdge::add_outerFacesToVector(std::vector<OuterFace*> *vectorOuterFaces)
{
    if (find(vectorOuterFaces->begin(), vectorOuterFaces->end(), m_outerFace1)==vectorOuterFaces->end())
    {
        vectorOuterFaces->push_back(m_outerFace1);
    }

    if (find(vectorOuterFaces->begin(), vectorOuterFaces->end(), m_outerFace2)==vectorOuterFaces->end())
    {
        vectorOuterFaces->push_back(m_outerFace2);
    }
}



// ---------- setters ----------
void OuterEdge::set_outerVertice1(OuterVertex* outerVertex)
{
    m_outerVertex1->remove_outerEdge(this);
    m_outerVertex1 = outerVertex;
    m_outerVertex1->add_outerEdge(this);
}
void OuterEdge::set_outerVertice2(OuterVertex* outerVertex)
{
    m_outerVertex2->remove_outerEdge(this);
    m_outerVertex2 = outerVertex;
    m_outerVertex2->add_outerEdge(this);
}

void OuterEdge::switch_outerVertices(OuterVertex* oldVertex, OuterVertex* newVertex)
{
    if (oldVertex == m_outerVertex1)
    {
        set_outerVertice1(newVertex);
    }
    else if (oldVertex == m_outerVertex2)
    {
        set_outerVertice2(newVertex);
    }
    else
    {
        std::cerr << "The outer edge does not have this outer vertex" << '\n';
    }
}
