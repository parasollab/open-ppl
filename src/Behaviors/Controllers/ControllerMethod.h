#ifndef CONTROLLER_METHOD_H_
#define CONTROLLER_METHOD_H_

#include <vector>

#include "MPProblem/Robot/Control.h"

class Cfg;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// Controllers determine the best control needed to move a robot from one
/// configuration to another. They require the standard Cfg as the planning space
/// representation because that's what we can support in simulation.
////////////////////////////////////////////////////////////////////////////////
class ControllerMethod {

  public:

    ///@name Construction
    ///@{

    ControllerMethod(Robot* const _r);

    virtual ~ControllerMethod() = default;

    ///@}
    ///@name Interface
    ///@{

    /// Find the best available control to steer a robot from a starting
    /// configuration to a target configuration.
    /// @param[in] _current The current configuration.
    /// @param[in] _target The target configuration.
    /// @param[in] _dt The timestep length.
    /// @return The best available control for steering from _current to _target.
    virtual Control operator()(const Cfg& _current, const Cfg& _target,
        const double _dt);

    ///@}

  protected:

    ///@name Control Selection
    ///@{

    /// Compute the desired generalized force to move from the current position
    /// to the target.
    /// @param[in] _current The current configuration.
    /// @param[in] _target The target configuration.
    /// @param[in] _dt The timestep length.
    /// @return The ideal generalized force.
    virtual std::vector<double> ComputeDesiredForce(const Cfg& _current,
        const Cfg& _target, const double _dt) = 0;

    /// Compute the control that produces the closest force to the ideal.
    /// @param[in] _current The current configuration.
    /// @param[in] _force The desired force.
    /// @return The control whos result is nearest to _force.
    virtual Control ComputeNearestControl(const Cfg& _current,
        std::vector<double>&& _force);

    /// Compute the continuous control that produces the closest force to the
    /// ideal.
    /// @param[in] _current The current configuration.
    /// @param[in] _force The desired force.
    /// @return The control whos result is nearest to _force.
    virtual Control ComputeNearestContinuousControl(const Cfg& _current,
        std::vector<double>&& _force);

    /// Compute the discrete control that produces the closest force to the
    /// ideal.
    /// @param[in] _current The current configuration.
    /// @param[in] _force The desired force.
    /// @return The control whos result is nearest to _force.
    virtual Control ComputeNearestDiscreteControl(const Cfg& _current,
        std::vector<double>&& _force);

    ///@}
    ///@name Internal State
    ///@{

    Robot* const m_robot; ///< The robot that owns this controller.

    ///@}

};

#endif
