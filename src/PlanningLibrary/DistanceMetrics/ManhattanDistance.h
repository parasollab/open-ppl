#ifndef MANHATTAN_DISTANCE_H_
#define MANHATTAN_DISTANCE_H_

#include "MinkowskiDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ManhattanDistance : public MinkowskiDistance<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;

    ManhattanDistance(bool _normalize = false);
    ManhattanDistance(MPProblemType* _problem, XMLNode& _node);
    virtual ~ManhattanDistance();
};

template<class MPTraits>
ManhattanDistance<MPTraits>::
ManhattanDistance(bool _normalize) :
  MinkowskiDistance<MPTraits>(1, 1, 1, _normalize) {
  this->SetName("Manhattan");
}

template<class MPTraits>
ManhattanDistance<MPTraits>::
ManhattanDistance(MPProblemType* _problem, XMLNode& _node) :
  MinkowskiDistance<MPTraits>(_problem, _node, false) {
    this->SetName("Manhattan");

    this->m_r1 = 1;
    this->m_r2 = 1;
    this->m_r3 = 1;
    this->m_normalize = _node.Read("normalize", false, false,
        "flag if position dof should be normalized by environment diagonal");
  }

template<class MPTraits>
ManhattanDistance<MPTraits>::
~ManhattanDistance() {
}

#endif
