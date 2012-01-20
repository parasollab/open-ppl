#ifndef SCALEDEUCLIDEANDISTANCE_H_
#define SCALEDEUCLIDEANDISTANCE_H_

#include "EuclideanDistance.h"



class ScaledEuclideanDistance : public EuclideanDistance {
  public:
    ScaledEuclideanDistance();
    ScaledEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~ScaledEuclideanDistance();
    virtual bool operator==(const ScaledEuclideanDistance& _dm) const;
    virtual void PrintOptions(ostream& _os) const;
    virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2);
    double GetS() const; 
    
  protected:
    double m_sValue;
  
};

#endif
