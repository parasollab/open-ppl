#ifndef PMPL_CENTER_OF_MASS_DISTANCE_H_
#define PMPL_CENTER_OF_MASS_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Measure the Euclidean distance between the center of mass for two
/// configurations.
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CenterOfMassDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    CenterOfMassDistance();
    CenterOfMassDistance(XMLNode& _node);

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("CenterOfMass");
}


template <typename MPTraits>
CenterOfMassDistance<MPTraits>::
CenterOfMassDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("CenterOfMass");
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
CenterOfMassDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  _c1.ConfigureRobot();
  const auto com1 = _c1.GetMultiBody()->GetCenterOfMass();
  _c2.ConfigureRobot();
  const auto com2 = _c2.GetMultiBody()->GetCenterOfMass();
  return (com1 - com2).norm();
}

/*----------------------------------------------------------------------------*/

#endif
