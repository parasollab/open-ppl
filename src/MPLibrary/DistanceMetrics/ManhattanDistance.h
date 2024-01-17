#ifndef PMPL_MANHATTAN_DISTANCE_H_
#define PMPL_MANHATTAN_DISTANCE_H_

#include "MinkowskiDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// Compute the Manhattan distance between two configurations.
///
/// The Manhattan distance requires that each basis of the configuration space
/// be traversed separately. Its name is analogous to how one moves around the
/// city of Manhattan by following the streets rather than flying over the
/// buildings.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class ManhattanDistance : virtual public MinkowskiDistance {

  public:

    ///@name Construction
    ///@{

    ManhattanDistance(bool _normalize = false);
    ManhattanDistance(XMLNode& _node);
    virtual ~ManhattanDistance() = default;

    ///@}

};

#endif
