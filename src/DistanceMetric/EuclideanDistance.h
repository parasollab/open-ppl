#ifndef EUCLIDEANDISTANCE_H_
#define EUCLIDEANDISTANCE_H_

#include "MinkowskiDistance.h"

class EuclideanDistance : public MinkowskiDistance {
  public:
    EuclideanDistance(bool _normalize = false);
    EuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~EuclideanDistance();
};

#endif
