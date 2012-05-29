#ifndef MANHATTANDISTANCE_H_
#define MANHATTANDISTANCE_H_

#include "MinkowskiDistance.h"

class ManhattanDistance : public MinkowskiDistance {
  public:
    ManhattanDistance(bool _normalize = false);
    ManhattanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~ManhattanDistance();
};

#endif
