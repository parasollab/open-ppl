#ifndef CENTER_OF_MASS_DISTANCE_H_
#define CENTER_OF_MASS_DISTANCE_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup DistanceMetrics
/// @brief Measure the Euclidean distance between the center of mass for two
///        configurations.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CenterOfMassDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
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
  return (_c1.GetRobotCenterofMass() - _c2.GetRobotCenterofMass()).norm();
}

/*----------------------------------------------------------------------------*/

#endif
