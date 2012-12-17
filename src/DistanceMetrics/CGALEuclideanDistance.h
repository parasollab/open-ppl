#ifndef CGALEUCLIDEANDISTANCE_H_
#define CGALEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"
#include "MPProblem/Environment.h"

template<class MPTraits> 
class CGALEuclideanDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    
    CGALEuclideanDistance();
    CGALEuclideanDistance(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~CGALEuclideanDistance();
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

template<class MPTraits>
CGALEuclideanDistance<MPTraits>::CGALEuclideanDistance() : DistanceMetricMethod<MPTraits>() {
  this->m_name = "CGALEuclidean";
}

template<class MPTraits>
CGALEuclideanDistance<MPTraits>::  
CGALEuclideanDistance(MPProblemType*_problem, XMLNodeReader& _node) : DistanceMetricMethod<MPTraits>(_problem, _node) {
  this->m_name = "CGALEuclidean";
}

template<class MPTraits>
CGALEuclideanDistance<MPTraits>::~CGALEuclideanDistance() {}

template<class MPTraits>
double 
CGALEuclideanDistance<MPTraits>::Distance(Environment* _env, 
    const Cfg& _c1, const Cfg& _c2) {
  double maxRange = 0.0;
  for(size_t i=0; i< _c1.PosDOF(); ++i) {
    pair<double,double> range = _env->GetBoundary()->GetRange(i);
    maxRange = max(maxRange, range.second-range.first);
  }
  
  double dist = 0.0;
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

#endif
