#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <set>

#include "MPProblem/ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Control/Control.h"

class ControlGenerator;
class Robot;
class SteeringFunction;


////////////////////////////////////////////////////////////////////////////////
/// Determines how to apply controls to achieve a higher-level motion, such as
/// 'move to this point'.
///
/// @details The working data for this object is a target configuration: the
///          controller will attempt to apply controls that lead toward the
///          target. When the target is reached, the status
///
/// @TODO Generalize the controller target to accept a constraint instead of a
///       configuration.
/// @TODO Extend the implementation to support path constraints that must be
///       respected during travel.
////////////////////////////////////////////////////////////////////////////////
class Controller {

  protected:

    ///@name Internal State
    ///@{

    Robot* const m_robot; ///< The robot that this object controls.

    bool m_ready{true};   ///< Is the controller ready to execute a new task?
    Cfg m_target;         ///< The current target.

    /// @TODO Optimize this by replacing with an unordered_set. Requires a
    ///       hashing function.
    std::set<Control> m_controls; ///< The available controls.

    SteeringFunction* m_steeringFunction{nullptr}; ///< The steering function.

    ///@}

  public:

    ///@name Construction
    ///@{

    Controller(Robot* const _r);
    virtual ~Controller();

    ///@}
    ///@name Controller Properties
    ///@{

    /// Compute the controls that this controller can use.
    /// @param[in] _g The control generator to use.
    void ComputeControls(const ControlGenerator& _g);

    /// Set the function that will be used to determine the generalized force
    /// required to steer the robot between configurations. The controller will
    /// take ownership of the steering function and delete it when necessary.
    /// @param[in] _f The steering function to use.
    void SetSteeringFunction(SteeringFunction* const _f);

    ///@}
    ///@name Planning Interface
    ///@{

    /// Get the available controls. These should be computed during the
    /// Controller's construction.
    const std::set<Control>& GetControls() const;

    /// Test the results of applying an control to a given state.
    /// @param[in] _c The control to apply.
    /// @param[in] _start The starting configuration.
    /// @param[in] _ticks The number of timesteps to take.
    /// @return The result of applying _c from _start.
    Cfg Test(const Control& _c, const Cfg& _start, const size_t _ticks) const;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Is the controller ready for new commands?
    /// @return True if the controller has completed its task, false otherwise.
    bool Ready() const noexcept;

    /// Set the target configuration. The controller will attempt to reach this
    /// target by sending commands to the actuators.
    /// @param[in] _cfg The new target configuration.
    void SetTarget(const Cfg& _c);

    /// Send commands to the simulated robot to drive it towards the target.
    void Step();

    ///@}

};

#endif
