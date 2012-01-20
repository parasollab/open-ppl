#ifndef BINARYLPSWEPTDISTANCE_H_
#define BINARYLPSWEPTDISTANCE_H_

#include "DistanceMetricMethod.h"


class BinaryLPSweptDistance : public DistanceMetricMethod {
  public:
    BinaryLPSweptDistance();
    BinaryLPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    BinaryLPSweptDistance(string _lp, double _posRes = 0.1, double _oriRes = 0.1, double _tolerance = 0.01, int _maxAttempts = 100, bool _bbox = false);
    ~BinaryLPSweptDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    virtual double DistanceCalc(Environment* _env, const Cfg& _c1, const Cfg& _c2, double _posRes, double _oriRes);
    double SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2);
    
    
  protected:
    string m_lp;
    double m_positionRes, m_orientationRes, m_tolerance;
    int m_maxAttempts,m_distCallsCount;
    bool m_useBbox;
};

#endif
