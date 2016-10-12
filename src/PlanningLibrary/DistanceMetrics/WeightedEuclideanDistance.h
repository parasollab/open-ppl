#ifndef WEIGHTED_EUCLIDEAN_DISTANCE_H_
#define WEIGHTED_EUCLIDEAN_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Weighted Euclidean distance for State space
///
/// Weighted Euclidean distance in State space will have four weight components:
/// position, rotation, velocity, and angular velocity.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class WeightedEuclideanDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType       StateType;

    ///@}
    ///@name Construction
    ///@{

    WeightedEuclideanDistance();
    WeightedEuclideanDistance(XMLNode& _node);
    virtual ~WeightedEuclideanDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const StateType& _s1, const StateType& _s2) override;

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
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
WeightedEuclideanDistance<MPTraits>::
Distance(const StateType& _s1, const StateType& _s2) {
  StateType diff = _s1 - _s2;

  Vector3d pos(diff[0], diff[1], diff[2]);
  Vector3d rot(diff[3], diff[4], diff[5]);
  const vector<double>& velocity = diff.GetVelocity();
  Vector3d vel(velocity[0], velocity[1], velocity[2]);
  Vector3d avl(velocity[3], velocity[4], velocity[5]);

  double dist = m_posW*pos.norm() + m_rotW*rot.norm() +
    m_velW*vel.norm() + m_avlW*avl.norm();
  return dist;
}

/*----------------------------------------------------------------------------*/

#endif
