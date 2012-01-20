#ifndef MINKOWSKIDISTANCE_H_
#define MINKOWSKIDISTANCE_H_

#include "DistanceMetricMethod.h"



class MinkowskiDistance : public DistanceMetricMethod {
  public:
    MinkowskiDistance();
    MinkowskiDistance(double _ri, double _rii, double _riii) : m_r1(_ri), m_r2(_rii), m_r3(_riii) {}
    MinkowskiDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~MinkowskiDistance();
    bool operator==(const MinkowskiDistance& _dm) const;
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    double GetR1() const;
    double GetR2() const;
    double GetR3() const;
    
  protected:
    /**Power factors for Minkowski Distance **/
    double m_r1; ///<For position part.
    double m_r2; ///<For rotation part.
    double m_r3; ///<For calculating root.
  
};

#endif
