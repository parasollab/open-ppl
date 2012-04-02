#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))

#ifndef REACHABLEDISTANCE_H_
#define REACHABLEDISTANCE_H_

#include "DistanceMetricMethod.h"


class ReachableDistance : public DistanceMetricMethod {
  public:
    ReachableDistance();
    ReachableDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~ReachableDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

#endif

#endif
