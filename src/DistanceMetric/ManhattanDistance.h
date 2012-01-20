#ifndef MANHATTANDISTANCE_H_
#define MANHATTANDISTANCE_H_

#include "DistanceMetricMethod.h"


/**This computes the manhattan distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class ManhattanDistance : public DistanceMetricMethod {
  public:
    ManhattanDistance();
    ManhattanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~ManhattanDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
  
};

#endif
