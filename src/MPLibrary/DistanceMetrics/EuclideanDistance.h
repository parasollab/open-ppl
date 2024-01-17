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
class EuclideanDistance : virtual public MinkowskiDistance {

  public:

    ///@name Construction
    ///@{

    EuclideanDistance(bool _normalize = false);
    EuclideanDistance(XMLNode& _node);
    virtual ~EuclideanDistance() = default;

    ///@}

};

#endif
