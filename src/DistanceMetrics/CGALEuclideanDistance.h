#ifndef CGALEUCLIDEANDISTANCE_H_
#define CGALEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"

class CGALEuclideanDistance : public DistanceMetricMethod {
  public:
    CGALEuclideanDistance();
    CGALEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~CGALEuclideanDistance();
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

#endif
