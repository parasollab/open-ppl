#ifndef WEIGHTED_EUCLIDEAN_DISTANCE_H_
#define WEIGHTED_EUCLIDEAN_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Weighted Euclidean distance for State space
/// @tparam MPTraits Motion planning universe
///
/// Weighted Euclidean distance in State space will have four weight components:
/// position, rotation, velocity, and angular velocity.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class WeightedEuclideanDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType StateType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    WeightedEuclideanDistance(double _posW = 0.25, double _rotW = 0.25,
        double _velW = 0.25, double _avlW = 0.25);
    WeightedEuclideanDistance(MPProblemType* _problem, XMLNode& _node);
    virtual ~WeightedEuclideanDistance() = default;

    virtual double Distance(const StateType& _s1, const StateType& _s2);

  private:
    double m_posW; ///< Positional weight
    double m_rotW; ///< Rotational weight
    double m_velW; ///< Velocity weight
    double m_avlW; ///< Angular velocity weight
};

template<class MPTraits>
WeightedEuclideanDistance<MPTraits>::
WeightedEuclideanDistance(double _posW, double _rotW, double _velW,
    double _avlW) :
  DistanceMetricMethod<MPTraits>(), m_posW(_posW), m_rotW(_rotW),
  m_velW(_velW), m_avlW(_avlW) {
    this->SetName("WeightedEuclidean");
  }

template<class MPTraits>
WeightedEuclideanDistance<MPTraits>::
WeightedEuclideanDistance(MPProblemType* _problem, XMLNode& _node) :
  DistanceMetricMethod<MPTraits>(_problem, _node) {
    this->SetName("WeightedEuclidean");

    m_posW = _node.Read("posWeight", true, 0.0, 0.0, 1.0, "Position weight");
    m_rotW = _node.Read("rotWeight", true, 0.0, 0.0, 1.0, "Rotation weight");
    m_velW = _node.Read("velWeight", true, 0.0, 0.0, 1.0, "Velocity weight");
    m_avlW = _node.Read("avlWeight", true, 0.0, 0.0, 1.0,
        "Angular velocity weight");
}

template<class MPTraits>
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

#endif
