#ifndef CENTER_OF_MASS_DISTANCE_H_
#define CENTER_OF_MASS_DISTANCE_H_

#include "DistanceMetricMethod.h"

template<class MPTraits>
class CenterOfMassDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    CenterOfMassDistance();
    CenterOfMassDistance(MPProblemType* _problem, XMLNodeReader& _node);

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);
};

template<class MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("CenterOfMass");
}

template<class MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance(MPProblemType* _problem, XMLNodeReader& _node) :
  DistanceMetricMethod<MPTraits>(_problem, _node, true) {
    this->SetName("CenterOfMass");
  }

template<class MPTraits>
double
CenterOfMassDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  return (_c1.GetRobotCenterofMass(env) - _c2.GetRobotCenterofMass(env)).norm();
}

#endif
