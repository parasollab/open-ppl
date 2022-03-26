#ifndef PMPL_EUCLIDEAN_DISTANCE_H_
#define PMPL_EUCLIDEAN_DISTANCE_H_

#include "MinkowskiDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// Measure the standard Euclidean distance between two configurations.
///
/// Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class KnotTheoryDistance : virtual public MinkowskiDistance<MPTraits> {

  public:

    ///@name Construction
    ///@{

    KnotTheoryDistance(bool _normalize = false);
    KnotTheoryDistance(XMLNode& _node);
    virtual ~KnotTheoryDistance() = default;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance(bool _normalize) :
    MinkowskiDistance<MPTraits>(2, 2, 1. / 2, _normalize) {
  this->SetName("KnotTheory");
}


template <typename MPTraits>
KnotTheoryDistance<MPTraits>::
KnotTheoryDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node), 
                                    MinkowskiDistance<MPTraits>(_node) {
  this->SetName("KnotTheory");

  this->m_r1 = 2;
  this->m_r2 = 2;
  this->m_r3 = 1.0/2;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/

#endif
