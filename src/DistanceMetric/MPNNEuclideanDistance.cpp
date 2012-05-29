#include "MPNNEuclideanDistance.h"

MPNNEuclideanDistance::MPNNEuclideanDistance() : DistanceMetricMethod() {
  m_name = "mpnnEuclidean";
}

MPNNEuclideanDistance::  
MPNNEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "mpnnEuclidean";
}

MPNNEuclideanDistance::~MPNNEuclideanDistance() {}

double MPNNEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double maxRange = 0.0;
  for(int i=0; i< _c1.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    maxRange = max(maxRange, range.second-range.first);
  }
  
  double dist = 0.0;
  for(int i=0; i<_c1.DOF(); ++i) {
    // calculate distance for positional coordinate
    if (i < _c1.PosDOF()) {
      double diff = 2*PI*(_c1.GetSingleParam(i) - _c2.GetSingleParam(i))/maxRange;
      dist += diff * diff;
    }
    else {// calculate distance for rotational coordinate
      double diff1 = 2*PI*(_c1.GetSingleParam(i) - _c2.GetSingleParam(i));
      if (diff1 < 0)
        diff1 *= -1;
      double diff2 = 2*PI - diff1;
      double diff = (diff1 < diff2) ? diff1 : diff2;
      dist += diff * diff;
    }
  }
  return sqrt(dist);
}

