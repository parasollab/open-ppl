//Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5
//This allows us to use Minkowski Distance to calculate Euclidean Distance

#ifndef EUCLIDEANDISTANCE_H_
#define EUCLIDEANDISTANCE_H_

#include "MinkowskiDistance.h"

template<class MPTraits>
class EuclideanDistance : public MinkowskiDistance<MPTraits> {
  public:
    EuclideanDistance(bool _normalize = false);
    EuclideanDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~EuclideanDistance();
};

template<class MPTraits>
EuclideanDistance<MPTraits>::EuclideanDistance(bool _normalize) : MinkowskiDistance<MPTraits>(2, 2, 1.0/2, _normalize) {
  this->m_name = "Euclidean";
}

template<class MPTraits>
EuclideanDistance<MPTraits>::EuclideanDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node, bool _warn) : 
  MinkowskiDistance<MPTraits>(_problem, _node, false, false) {
    this->m_name = "Euclidean";

    this->m_r1 = 2;
    this->m_r2 = 2;
    this->m_r3 = 1.0/2;
    this->m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");

    if(_warn)
      _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
EuclideanDistance<MPTraits>::~EuclideanDistance() {}

#endif
