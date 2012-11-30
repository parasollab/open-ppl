#ifndef BINARYLPSWEPTDISTANCE_H_
#define BINARYLPSWEPTDISTANCE_H_

#include "LPSweptDistance.h"

class BinaryLPSweptDistance : public LPSweptDistance {
  public:
    BinaryLPSweptDistance();
    BinaryLPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    BinaryLPSweptDistance(string _lp, double _posRes = 0.1, double _oriRes = 0.1, double _tolerance = 0.01, int _maxAttempts = 100, bool _bbox = false);
    virtual ~BinaryLPSweptDistance();

    virtual void PrintOptions(ostream& _os) const;
    
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    
  protected:
    double m_tolerance;
    int m_maxAttempts;
};

#endif

