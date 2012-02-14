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

double 
EuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  dist = sqrt(2.0)*ScaledDistance<CfgType>(_env,_c1, _c2, 0.5);
  return dist;
}

double 
EuclideanDistance::ScaledDistanceImpl(Environment* _env,const Cfg& _c1, const Cfg& _c2, double _sValue, Cfg& tmp) {
  tmp.subtract(_c1,_c2);
  double posMag(0.0);
  double maxRange(0.0);
  for(int i=0; i< tmp.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) maxRange = tmpRange;
  }
  for(int i=0; i< tmp.PosDOF(); ++i) {
     posMag += sqr(tmp.GetSingleParam(i) / maxRange);
  }
  double dReturn = sqrt(_sValue*posMag + (1.0 - _sValue)*sqr(tmp.OrientationMagnitude()) );
  return dReturn;
}

void EuclideanDistance::ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _norm) {
  double originalLength = this->Distance(_env, _o, _c);
  double diff;
  do {
    for(int i=0; i<_c.DOF(); ++i)
      _c.SetSingleParam(i, (_length/originalLength)*_c.GetSingleParam(i), _norm);
    originalLength = this->Distance(_env, _o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1)); 
}
