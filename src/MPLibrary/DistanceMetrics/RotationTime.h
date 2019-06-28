#ifndef PMPL_ROTATION_TIME_H_
#define PMPL_rOTATION_TIME_H_

#include "DistanceMetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Finds the maximum rotation between two manipulator configurations. This is
/// used to determine the time taken to move between configurations. It does
/// assume that all joints move at the same speed (extension of the hack where 
/// time and distance are used interchangeably).
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RotationTime : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;

    ///@}
    ///@name Construction
    ///@{

    RotationTime();
    RotationTime(XMLNode& _node);
    virtual ~RotationTime() = default;

    ///@}
    ///@name DistanceMetricMethod Overrides
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

		///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
RotationTime<MPTraits>::
RotationTime() {
	this->SetName("RotationTime");
}


template <typename MPTraits>
RotationTime<MPTraits>::
RotationTime(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("RotationTime");
	
	//TODO::Set up an input vector for rotation speed of each dof to convert from radians 
	// to actual time instead of relying on assumption that all rotations occur at the 
	// same speed.
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

template <typename MPTraits>
double
RotationTime<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
	double max = 0.0;
	for(size_t i = 0; i < _c1.DOF(); i++) {
		double abs = std::abs(_c1[i] - _c2[i]);
		if(abs > max)
			max = abs;
	}
	return max;
}



/*----------------------------------------------------------------------------*/

#endif
