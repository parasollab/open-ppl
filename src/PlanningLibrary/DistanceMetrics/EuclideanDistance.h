#ifndef EUCLIDEAN_DISTANCE_H_
#define EUCLIDEAN_DISTANCE_H_

#include "MinkowskiDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5
/// This allows us to use Minkowski Distance to calculate Euclidean Distance
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class EuclideanDistance : public MinkowskiDistance<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;

    EuclideanDistance(bool _normalize = false);
    EuclideanDistance(MPProblemType* _problem, XMLNode& _node);
};

template<class MPTraits>
EuclideanDistance<MPTraits>::
EuclideanDistance(bool _normalize) :
  MinkowskiDistance<MPTraits>(2, 2, 1.0/2, _normalize) {
    this->SetName("Euclidean");
}

template<class MPTraits>
EuclideanDistance<MPTraits>::
EuclideanDistance(MPProblemType* _problem, XMLNode& _node) :
  MinkowskiDistance<MPTraits>(_problem, _node, false) {
    this->SetName("Euclidean");

    this->m_r1 = 2;
    this->m_r2 = 2;
    this->m_r3 = 1.0/2;
    this->m_normalize = _node.Read("normalize", false, false,
        "flag if position dof should be normalized by environment diagonal");
}

#endif
