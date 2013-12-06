#ifndef MANHATTANDISTANCE_H_
#define MANHATTANDISTANCE_H_

#include "MinkowskiDistance.h"

template<class MPTraits>
class ManhattanDistance : public MinkowskiDistance<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;

    ManhattanDistance(bool _normalize = false);
    ManhattanDistance(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~ManhattanDistance();
};

template<class MPTraits>
ManhattanDistance<MPTraits>::ManhattanDistance(bool _normalize) :
  MinkowskiDistance<MPTraits>(1, 1, 1, _normalize){
  this->SetName("Manhattan");
}

template<class MPTraits>
ManhattanDistance<MPTraits>::ManhattanDistance(MPProblemType* _problem,
    XMLNodeReader& _node) :
  MinkowskiDistance<MPTraits>(_problem, _node, false, false) {
    this->SetName("Manhattan");

    this->m_r1 = 1;
    this->m_r2 = 1;
    this->m_r3 = 1;
    this->m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");

    _node.warnUnrequestedAttributes();
  }

template<class MPTraits>
ManhattanDistance<MPTraits>::~ManhattanDistance() {
}

#endif
