#ifndef DISTANCEMETRICMETHOD_H
#define DISTANCEMETRICMETHOD_H

#include "Roadmap.h"

using namespace std;

class MPProblem;

const double MAX_DIST =  1e10;

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod  : public MPBaseObject {
 public:
  DistanceMetricMethod();
  DistanceMetricMethod(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
  virtual ~DistanceMetricMethod();

  string GetName() const {return m_name;}
  
  virtual void PrintOptions(ostream& _os) const;
  
  virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) = 0;
  virtual void ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _norm=true);

 protected:
  string m_name;
};

ostream& operator<< (ostream& _os, const DistanceMetricMethod& _dm);

#endif
