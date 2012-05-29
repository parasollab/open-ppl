#include "ScaledEuclideanDistance.h"

ScaledEuclideanDistance::ScaledEuclideanDistance(double _scale, bool _normalize) : EuclideanDistance(_normalize) {
  m_name = "scaledEuclidean";
}

ScaledEuclideanDistance::
ScaledEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : EuclideanDistance(_node, _problem, false) {
  m_name = "scaledEuclidean";
  
  m_scale = _node.numberXMLParameter("scale", false, 0.5, 0.0, 1.0, "scale factor");

  if(_warn)
    _node.warnUnrequestedAttributes();
}

ScaledEuclideanDistance::~ScaledEuclideanDistance() {
}

void ScaledEuclideanDistance::PrintOptions(ostream& _os) const {
  EuclideanDistance::PrintOptions(_os);
  _os << " scale=" << m_scale;
}

double ScaledEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  Cfg* c = DifferenceCfg<CfgType>(_c1, _c2);
  double result = pow(m_scale*PositionDistance(_env, *c) + (1-m_scale)*OrientationDistance(*c), m_r3);
  delete c;
  return result;
}
