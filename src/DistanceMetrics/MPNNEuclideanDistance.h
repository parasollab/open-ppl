#ifndef MPNNEUCLIDEANDISTANCE_H_
#define MPNNEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"

template<class MPTraits>
class MPNNEuclideanDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    
    MPNNEuclideanDistance();
    MPNNEuclideanDistance(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MPNNEuclideanDistance();
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

template<class MPTraits>
MPNNEuclideanDistance<MPTraits>::MPNNEuclideanDistance() : DistanceMetricMethod<MPTraits>() {
  this->m_name = "MPNNEuclidean";
}

template<class MPTraits>
MPNNEuclideanDistance<MPTraits>::  
MPNNEuclideanDistance(MPProblemType* _problem, XMLNodeReader& _node) : DistanceMetricMethod<MPTraits>(_problem, _node) {
  this->m_name = "MPNNEuclidean";
}

template<class MPTraits>
MPNNEuclideanDistance<MPTraits>::~MPNNEuclideanDistance() {}

template<class MPTraits>
double 
MPNNEuclideanDistance<MPTraits>::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double maxRange = 0.0;
  for(size_t i=0; i< _c1.PosDOF(); ++i) {
    std::pair<double,double> range = _env->GetBoundary()->GetRange(i);
    maxRange = max(maxRange, range.second-range.first);
  }
  
  double dist = 0.0;
  for(size_t i=0; i<_c1.DOF(); ++i) {
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

#endif
