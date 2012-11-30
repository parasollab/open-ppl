#include "CGALEuclideanDistance.h"

CGALEuclideanDistance::CGALEuclideanDistance() : DistanceMetricMethod() {
  m_name = "cgalEuclidean";
}

CGALEuclideanDistance::  
CGALEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "cgalEuclidean";
}

CGALEuclideanDistance::~CGALEuclideanDistance() {}

double CGALEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double maxRange = 0.0;
  for(size_t i=0; i< _c1.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundary()->GetRange(i);
    maxRange = max(maxRange, range.second-range.first);
  }
  
  double dist(0.0);
  for(size_t i=0; i<_c1.DOF(); ++i) {
    // calculate distance for positional coordinate
    if (i < _c1.PosDOF()) {
      double diff = (_c1.GetSingleParam(i) - _c2.GetSingleParam(i))/maxRange;
      dist += diff * diff;
    }
    else {// calculate distance for rotational coordinate
      double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
      dist += diff * diff;
    }
  }
  return sqrt(dist);
}

