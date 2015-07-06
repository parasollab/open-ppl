#ifndef CENTER_OF_MASS_DISTANCE_H_
#define CENTER_OF_MASS_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief TODO.
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CenterOfMassDistance : public DistanceMetricMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    CenterOfMassDistance();
    CenterOfMassDistance(MPProblemType* _problem, XMLNode& _node);

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);
};

template<class MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("CenterOfMass");
}

template<class MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance(MPProblemType* _problem, XMLNode& _node) :
  DistanceMetricMethod<MPTraits>(_problem, _node) {
    this->SetName("CenterOfMass");
  }

template<class MPTraits>
double
CenterOfMassDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  return (_c1.GetRobotCenterofMass() - _c2.GetRobotCenterofMass()).norm();
}

#endif
