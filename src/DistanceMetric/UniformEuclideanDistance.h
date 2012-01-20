#ifndef UNIFORMEUCLIDEANDISTANCE_H_
#define UNIFORMEUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"


class UniformEuclideanDistance : public DistanceMetricMethod {
  public:
    UniformEuclideanDistance(bool _useRotational = false);
    virtual ~UniformEuclideanDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    
    
  protected:
    bool m_useRotational;
  
};

#endif
