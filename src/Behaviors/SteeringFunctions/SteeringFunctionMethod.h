#ifndef STEERING_FUNCTION_METHOD_H_
#define STEERING_FUNCTION_METHOD_H_

#include "MPProblem/Robot/Control.h"


////////////////////////////////////////////////////////////////////////////////
/// Steering functions determine the best control(s) needed to move a robot
/// from one configuration to another.
///
/// @details This method group is external to MPLibrary so that we can use its
///          derived classes in simulations without planning. This might be
///          useful for controlling dynamic obstacles or for implementing
///          reactionary agents.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SteeringFunctionMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    virtual ~SteeringFunctionMethod() = default;

    ///@}
    ///@name Interface
    ///@{

    /// Find the best available control to steer a robot from a starting
    /// configuration to a target configuration.
    /// @param[in] _current The start configuration.
    /// @param[in] _target The target configuration.
    /// @return The best available control for steering from _current to _target.
    virtual Control operator()(const CfgType& _current, const CfgType& _target)
        = 0;

    ///@}

};

#endif
