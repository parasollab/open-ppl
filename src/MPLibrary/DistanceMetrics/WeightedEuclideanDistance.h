#ifndef PMPL_WEIGHTED_EUCLIDEAN_DISTANCE_H_
#define PMPL_WEIGHTED_EUCLIDEAN_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Weighted Euclidean distance in State space will have four weight components:
/// position, rotation, velocity, and angular velocity.
///
/// @todo This doesn't work properly for linked robots. An additional weighting
///       is required for the joint space, which is currently wrapped up with
///       orientation.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class WeightedEuclideanDistance : virtual public DistanceMetricMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    WeightedEuclideanDistance();

    WeightedEuclideanDistance(XMLNode& _node);

    WeightedEuclideanDistance(const double _pos, const double _rot,
        const double _vel, const double _avl);

    virtual ~WeightedEuclideanDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_posW{.25}; ///< Position weight.
    double m_rotW{.25}; ///< Rotation weight.
    double m_velW{.25}; ///< Linear velocity weight.
    double m_avlW{.25}; ///< Angular velocity weight.

    ///@}
};

#endif
