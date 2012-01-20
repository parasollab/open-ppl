#include "EuclideanDistance.h"


EuclideanDistance::EuclideanDistance() : DistanceMetricMethod() {
  m_name = "euclidean";
}

EuclideanDistance::
EuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn){
  m_name = "euclidean";
}

EuclideanDistance::~EuclideanDistance() {
}

void EuclideanDistance::PrintOptions(ostream& _os) const{
  _os << "    " << GetName() << "::  " << endl;
}

double EuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  dist = sqrt(2.0)*ScaledDistance(_env,_c1, _c2, 0.5);
  return dist;
}

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
double EuclideanDistance::ScaledDistance(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue) {
  Cfg_free_tree c1Linkage(_c1.GetData());
  Cfg_free_tree c2Linkage(_c2.GetData());
  return ScaledDistanceImpl(_env, c1Linkage, c2Linkage, _sValue);
}
#else
double EuclideanDistance::ScaledDistance(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue) {
  return ScaledDistanceImpl(_env, _c1, _c2, _sValue);
}
#endif

double EuclideanDistance::ScaledDistanceImpl(Environment* _env,const Cfg& _c1, const Cfg& _c2, double _sValue) {
  Cfg *pTmp = _c1.CreateNewCfg();
  pTmp->subtract(_c1,_c2);
  double posMag(0.0);
  double maxRange(0.0);
  for(int i=0; i< pTmp->PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) maxRange = tmpRange;
  }
  for(int i=0; i< pTmp->PosDOF(); ++i) {
     posMag += sqr(pTmp->GetSingleParam(i) / maxRange);
  }
  double dReturn = sqrt(_sValue*posMag + (1.0 - _sValue)*sqr(pTmp->OrientationMagnitude()) );
  delete pTmp;
  return dReturn;
}

void EuclideanDistance::ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c) {
  double originalLength = this->Distance(_env, _o, _c);
  double diff;
  do {
    for(int i=0; i<_c.DOF(); ++i)
      _c.SetSingleParam(i, (_length/originalLength)*_c.GetSingleParam(i));
    originalLength = this->Distance(_env, _o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1)); 
}
