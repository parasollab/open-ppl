#ifndef EUCLIDEANDISTANCE_H_
#define EUCLIDEANDISTANCE_H_

#include "DistanceMetricMethod.h"

/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
  public:
    EuclideanDistance();
    EuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~EuclideanDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    virtual void ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _norm=true);
    
  protected:
    virtual double ScaledDistance(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue);
    double ScaledDistanceImpl(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _sValue);
  
};

#endif
