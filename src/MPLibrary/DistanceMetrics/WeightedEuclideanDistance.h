#ifndef WEIGHTED_EUCLIDEAN_DISTANCE_H_
#define WEIGHTED_EUCLIDEAN_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Weighted Euclidean distance in State space will have four weight components:
/// position, rotation, velocity, and angular velocity.
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class WeightedEuclideanDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

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

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

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

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
WeightedEuclideanDistance<MPTraits>::
WeightedEuclideanDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("WeightedEuclidean");
}


template <typename MPTraits>
WeightedEuclideanDistance<MPTraits>::
WeightedEuclideanDistance(XMLNode& _node) :
    DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("WeightedEuclidean");

  m_posW = _node.Read("posWeight", true, m_posW, 0.0, 1.0, "Position weight");
  m_rotW = _node.Read("rotWeight", true, m_rotW, 0.0, 1.0, "Rotation weight");
  m_velW = _node.Read("velWeight", true, m_velW, 0.0, 1.0,
      "Linear velocity weight");
  m_avlW = _node.Read("avlWeight", true, m_avlW, 0.0, 1.0,
      "Angular velocity weight");

  // Normalize weights.
  const double sum = m_posW + m_rotW + m_velW + m_avlW;
  if(sum <= 0)
    throw ParseException(_node.Where(), "Sum of weights are non-positive.");

  m_posW /= sum;
  m_rotW /= sum;
  m_velW /= sum;
  m_avlW /= sum;
}


template <typename MPTraits>
WeightedEuclideanDistance<MPTraits>::
WeightedEuclideanDistance(const double _pos, const double _rot,
                          const double _vel, const double _avl) {
  this->SetName("WeightedEuclidean");

  m_posW = _pos;
  m_rotW = _rot;
  m_velW = _vel;
  m_avlW = _avl;

  // Normalize weights.
  const double sum = m_posW + m_rotW + m_velW + m_avlW;
  if(sum <= 0)
    throw ParseException(WHERE, "Sum of weights are non-positive.");

  m_posW /= sum;
  m_rotW /= sum;
  m_velW /= sum;
  m_avlW /= sum;
}
/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
WeightedEuclideanDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  const CfgType diff = _c1 - _c2;

  return m_posW * diff.GetLinearPosition().norm()
       + m_rotW * (diff.GetAngularPosition() / PI).norm()
       + m_velW * diff.GetLinearVelocity().norm()
       + m_avlW * diff.GetAngularVelocity().norm();
}

/*----------------------------------------------------------------------------*/

#endif
