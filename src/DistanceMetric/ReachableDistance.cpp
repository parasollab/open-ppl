#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))

#include "ReachableDistance.h"
#include "CfgTypes.h"


ReachableDistance::ReachableDistance(double _s1, double _s2) : DistanceMetricMethod(), m_scale1(_s1), m_scale2(_s2)  {
  m_name = "reachable";
}
  
ReachableDistance::
ReachableDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, false) {
  m_name = "reachable";

  m_scale1 = _node.numberXMLParameter("s1", false, 0.33, 0.0, 1.0, "position scale factor");
  m_scale2 = _node.numberXMLParameter("s2", false, 0.33, 0.0, 1.0, "link lengths scale factor");

  if(m_scale1 + m_scale2 > 1.0) {
    cerr << "\n\nERROR in ReachableDistance(): scale factor parameters must add up to less than or equal to 1: " << m_scale1 << " + " << m_scale2 << " = " << m_scale1 + m_scale2 << "\nexiting.\n";
    exit(-1);
  }

  if(_warn)
    _node.warnUnrequestedAttributes();
}

ReachableDistance::~ReachableDistance() {}

void ReachableDistance::PrintOptions(ostream& _os) const {
  DistanceMetricMethod::PrintOptions(_os);
  _os << "s1=" << m_scale1 << " s2=" << m_scale2;
}

double ReachableDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  //get the position difference
  double dPosition = 0.0;
  vector<double> v1 = _c1.GetData();
  vector<double> v2 = _c2.GetData();
  for(size_t i=0; i<_c1.PosDOF(); ++i) {
    pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    dPosition += sqr(fabs(v1[i] - v2[i])/(range.second-range.first));
  }
  dPosition = sqrt(dPosition);
  
  //get the length difference
  double dLength = ((CfgType)_c1).LengthDistance((CfgType)_c2);
  
  //get the orientation difference
  double dOri = ((CfgType)_c1).OrientationDistance((CfgType)_c2);
  
  return (m_scale1*dPosition + m_scale2*dLength + (1-m_scale1-m_scale2)*dOri);
}

#endif
