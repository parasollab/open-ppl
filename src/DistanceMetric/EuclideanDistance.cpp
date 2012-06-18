#include "EuclideanDistance.h"

//Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5
//This allows us to use Minkowski Distance to calculate Euclidean Distance
EuclideanDistance::EuclideanDistance(bool _normalize) : MinkowskiDistance(2, 2, 1.0/2, _normalize) {
  m_name = "euclidean";
}

EuclideanDistance::EuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : 
  MinkowskiDistance(_node, _problem, false, false) {
    m_name = "euclidean";

    this->m_r1 = 2;
    this->m_r2 = 2;
    this->m_r3 = 1.0/2;
    this->m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

EuclideanDistance::~EuclideanDistance() {
}

