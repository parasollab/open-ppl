#ifndef CENTEROFMASSDISTANCE_H_
#define CENTEROFMASSDISTANCE_H_

#include "DistanceMetricMethod.h"

template<class MPTraits>
class CenterOfMassDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    CenterOfMassDistance();
    CenterOfMassDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~CenterOfMassDistance();

    virtual double Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2);
};

template<class MPTraits>
CenterOfMassDistance<MPTraits>::CenterOfMassDistance() : DistanceMetricMethod<MPTraits>() {
  this->m_name = "CenterOfMass";
}

template<class MPTraits>
CenterOfMassDistance<MPTraits>::CenterOfMassDistance(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : 
  DistanceMetricMethod<MPTraits>(_problem, _node, true) {
    this->m_name = "CenterOfMass";
  }

template<class MPTraits>
CenterOfMassDistance<MPTraits>::~CenterOfMassDistance() {
}

template<class MPTraits>
double 
CenterOfMassDistance<MPTraits>::Distance(Environment* _env, const CfgType& _c1, const CfgType& _c2) {
  return (_c1.GetRobotCenterofMass(_env) - _c2.GetRobotCenterofMass(_env)).norm();
}

#endif
