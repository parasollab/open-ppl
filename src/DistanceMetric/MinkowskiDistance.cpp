#include "MinkowskiDistance.h"

MinkowskiDistance::MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) : DistanceMetricMethod(), m_r1(_r1), m_r2(_r2), m_r3(_r3), m_normalize(_normalize) {
  m_name = "minkowski";
}

MinkowskiDistance::
MinkowskiDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn, bool _parse) : DistanceMetricMethod(_node, _problem, false) {
  m_name = "minkowski";
  
  if(_parse) {
    m_r1 = _node.numberXMLParameter("r1", false, 3.0, 0.0, 1000.0, "r1");
    m_r2 = _node.numberXMLParameter("r2", false, 3.0, 0.0, 1000.0, "r2");
    m_r3 = _node.numberXMLParameter("r3", false, 1.0/3.0, 0.0, 1000.0, "r3");
    m_normalize = _node.boolXMLParameter("normalize", false, false, "flag if position dof should be normalized by environment diagonal");
  }
  
  if(_warn)
    _node.warnUnrequestedAttributes();
}

MinkowskiDistance::~MinkowskiDistance() {}

void MinkowskiDistance::PrintOptions(ostream& _os) const {
  DistanceMetricMethod::PrintOptions(_os);
  _os << "r1=" << m_r1 << " r2=" << m_r2 << " r3=" << m_r3 << " normalize=" << m_normalize;
}

double MinkowskiDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  Cfg* pC = DifferenceCfg<CfgType>(_c1, _c2);
  double pos = PositionDistance(_env, *pC);
  double orient = OrientationDistance(*pC);
  delete pC;
  return pow(pos+orient, m_r3);
}

void MinkowskiDistance::ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _normalizeOrientation) {
  double originalLength = this->Distance(_env, _o, _c);
  double diff = _length - originalLength;
  do {
    for(int i=0; i<_c.DOF(); ++i)
      _c.SetSingleParam(i, (_length/originalLength)*_c.GetSingleParam(i), _normalizeOrientation);
    originalLength = this->Distance(_env, _o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

double MinkowskiDistance::PositionDistance(Environment* _env, const Cfg& _c) {
  double diagonal = 0;
  for(int i=0; i<_c.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    diagonal += pow(fabs(range.second-range.first), m_r1);
  }
  diagonal = pow(diagonal, m_r3);

  vector<double> p = _c.GetPosition();
  double pos = 0;
  for(size_t i=0; i<p.size(); ++i) 
    if(m_normalize)
      pos += pow(fabs(p[i])/diagonal, m_r1);
    else
      pos += pow(fabs(p[i]), m_r1);
  return pos; 
}

double MinkowskiDistance::OrientationDistance(const Cfg& _c) {
  vector<double> o = _c.GetOrientation();
  double orient = 0;
  for(size_t i=0; i<o.size(); ++i) 
    orient += pow(fabs(o[i]), m_r2);
  return orient;
}

