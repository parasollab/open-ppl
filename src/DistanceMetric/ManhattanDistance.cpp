#include "ManhattanDistance.h"

//Manhattan Distance is Minkowski Distance where r1=r2=r3=1
//This allows us to use Minkowski Distance to calculate Manhattan Distance
ManhattanDistance::ManhattanDistance(bool _normalize) : MinkowskiDistance(1, 1, 1, _normalize) {
  m_name = "manhattan";
}

ManhattanDistance::ManhattanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : 
  MinkowskiDistance(_node, _problem, false, false) {
    m_name = "manhattan";

    this->m_r1 = 1;
    this->m_r2 = 1;
    this->m_r3 = 1;
    this->m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

ManhattanDistance::~ManhattanDistance() {
}

