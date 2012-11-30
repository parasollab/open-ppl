#ifndef MPNNEUCLIDEANDISTANCE_H_
#define MPNNEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"

class MPNNEuclideanDistance : public DistanceMetricMethod {
  public:
    MPNNEuclideanDistance();
    MPNNEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~MPNNEuclideanDistance();
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

#endif
