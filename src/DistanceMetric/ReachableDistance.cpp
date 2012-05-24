#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))

#include "ReachableDistance.h"
#include "CfgTypes.h"


ReachableDistance::ReachableDistance() {
  m_name = "reachable";
}
  
ReachableDistance::
ReachableDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "reachable";
}

ReachableDistance::~ReachableDistance() {}

void ReachableDistance::PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << "::  " << endl;
}

double ReachableDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  //later make input param
  double s1 = 0.33;
  double s2 = 0.33;
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
  return (s1*dPosition + s2*dLength + (1-s1-s2)*dOri);
}

#endif
