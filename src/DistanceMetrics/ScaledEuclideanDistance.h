#ifndef SCALEDEUCLIDEANDISTANCE_H_
#define SCALEDEUCLIDEANDISTANCE_H_

#include "EuclideanDistance.h"

class ScaledEuclideanDistance : public EuclideanDistance {
  public:
    ScaledEuclideanDistance(double _scale = 0.5, bool _normalize = false);
    ScaledEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~ScaledEuclideanDistance();

    void PrintOptions(ostream& _os) const;

    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);

  protected:
    double m_scale;
};

#endif

