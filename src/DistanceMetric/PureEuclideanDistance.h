#ifndef PUREEUCLIDEANDISTANCE_H_
#define PUREEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"

class PureEuclideanDistance : public DistanceMetricMethod {
  public:
    PureEuclideanDistance();
    PureEuclideanDistance(string _label, int _useRotational);
    virtual ~PureEuclideanDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
  
};

#endif
