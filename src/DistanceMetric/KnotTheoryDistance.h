#ifndef KNOTTHEORYDISTANCE_H_
#define KNOTTHEORYDISTANCE_H_

#include "EuclideanDistance.h"


/**This computes the knot theory distance between two cfgs.  This class is 
  *derived off of Euclidean Distance.
  */
class KnotTheoryDistance : public EuclideanDistance {
  public:
    KnotTheoryDistance();
    KnotTheoryDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    ~KnotTheoryDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
};

#endif
