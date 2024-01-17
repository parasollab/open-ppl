#include "MinkowskiDistance.h"

/*------------------------------- Construction -------------------------------*/

MinkowskiDistance::
MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) :
    DistanceMetricMethod(), m_r1(_r1), m_r2(_r2), m_r3(_r3),
    m_normalize(_normalize) {
  this->SetName("Minkowski");
}


MinkowskiDistance::
MinkowskiDistance(XMLNode& _node) : DistanceMetricMethod(_node),
    m_r1(3), m_r2(3), m_r3(1./3.), m_normalize(false) {
  this->SetName("Minkowski");

  m_r1 = _node.Read("r1", false, 3., 0., MAX_DBL, "r1");
  m_r2 = _node.Read("r2", false, 3., 0., MAX_DBL, "r2");
  m_r3 = _node.Read("r3", false, 1. / 3., 0., MAX_DBL, "r3");
  m_normalize = _node.Read("normalize", false, false, "flag if position dof "
      "should be normalized by environment diagonal");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
MinkowskiDistance::
Print(std::ostream& _os) const {
  DistanceMetricMethod::Print(_os);
  _os << "\tr1 = " << m_r1 << endl
      << "\tr2 = " << m_r2 << endl
      << "\tr3 = " << m_r3 << endl
      << "\tnormalize = " << m_normalize << endl;
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

double
MinkowskiDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  const Cfg diff = _c2 - _c1;
  const double pos = PositionDistance(diff),
               ori = OrientationDistance(diff);
  return std::pow(pos + ori, m_r3);
}


void
MinkowskiDistance::
ScaleCfg(double _length, Cfg& _c, const Cfg& _o) {
  /// @todo This implementation is very poor. Scaling should be a
  ///       constant-time operation - complexity should not depend on the
  ///       length of the input vector.
  double originalLength = this->Distance(_o, _c);
  double diff = _length - originalLength;
  do {
    _c = (_c - _o) * (_length / originalLength) + _o;
    originalLength = this->Distance(_o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

/*---------------------------------- Helpers ---------------------------------*/

double
MinkowskiDistance::
PositionDistance(const Cfg& _c) {
  const std::vector<double> p = _c.GetPosition();
  double distance = 0;

  if(m_normalize) {
    const double diagonal = this->GetEnvironment()->GetBoundary()->GetMaxDist(
        m_r1, m_r3);
    for(size_t i = 0; i < p.size(); ++i)
      distance += std::pow(fabs(p[i]) / diagonal, m_r1);
  }
  else
    for(size_t i = 0; i < p.size(); ++i)
      distance += std::pow(fabs(p[i]), m_r1);

  return distance;
}


double
MinkowskiDistance::
OrientationDistance(const Cfg& _c) {
  const std::vector<double> o = _c.GetOrientation();
  double distance = 0;
  for(size_t i = 0; i < o.size(); ++i)
    distance += std::pow(fabs(o[i]), m_r2);
  return distance;
}

/*----------------------------------------------------------------------------*/
