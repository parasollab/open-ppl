#include "WeightedEuclideanDistance.h"

/*------------------------------- Construction -------------------------------*/

WeightedEuclideanDistance::
WeightedEuclideanDistance() : DistanceMetricMethod() {
  this->SetName("WeightedEuclidean");
}


WeightedEuclideanDistance::
WeightedEuclideanDistance(XMLNode& _node) :
    DistanceMetricMethod(_node) {
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


WeightedEuclideanDistance::
WeightedEuclideanDistance(const double _pos, const double _rot,
    const double _vel, const double _avl) {
  this->SetName("WeightedEuclidean");

  // Normalize weights.
  const double sum = _pos + _rot + _vel + _avl;
  if(sum <= 0)
    throw ParseException(WHERE) << "Sum of weights is non-positive.";


  m_posW = _pos / sum;
  m_rotW = _rot / sum;
  m_velW = _vel / sum;
  m_avlW = _avl / sum;
}

/*----------------------------- Distance Interface ---------------------------*/

double
WeightedEuclideanDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  const Cfg diff = _c2 - _c1;

  return m_posW * diff.GetLinearPosition().norm()
       + m_rotW * (diff.GetAngularPosition() / PI).norm()
       + m_velW * diff.GetLinearVelocity().norm()
       + m_avlW * diff.GetAngularVelocity().norm();
}
