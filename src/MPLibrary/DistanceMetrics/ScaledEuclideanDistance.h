#ifndef PMPL_SCALED_EUCLIDEAN_DISTANCE_H_
#define PMPL_SCALED_EUCLIDEAN_DISTANCE_H_

#include "EuclideanDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// A Euclidean distance that supports non-uniform weighting of the positional
/// and rotational components. Joint components are lumped in with orientation.
///
/// @todo Separate the computation of orientation and joint distances.
///
/// @todo Replace with WeightedEuclidean, which full encompases this class and
///       offers more functionality.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class ScaledEuclideanDistance : virtual public EuclideanDistance {

  public:

    ///@name Local Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    ScaledEuclideanDistance();
    ScaledEuclideanDistance(XMLNode& _node);
    virtual ~ScaledEuclideanDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_scale{.5}; ///< The ratio of positional to rotational weighting.

    ///@}
};

#endif
