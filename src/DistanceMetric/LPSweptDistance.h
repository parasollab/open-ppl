#ifndef LPSWEPTDISTANCE_H_
#define LPSWEPTDISTANCE_H_

#include "DistanceMetricMethod.h"



class LPSweptDistance : public DistanceMetricMethod {
  public:
    LPSweptDistance();
    LPSweptDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    LPSweptDistance(string _lp, double _positionRes = 0.1, double _orientationRes = 0.1, bool _bbox = false);
    ~LPSweptDistance();
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    double SweptDistance(const vector<GMSPolyhedron>& _poly1, const vector<GMSPolyhedron>& _poly2);
    
    
  protected:
    string m_lp;
    double m_positionRes, m_orientationRes;
    bool m_useBbox;
};

#endif
