#ifndef DYNAMICS_MODEL_H_
#define DYNAMICS_MODEL_H_

#include <vector>

class btMultiBody;
class Cfg;
class Control;
class InternalSimulator;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A dynamics model for a robot.
///
/// This model represents two things:
/// 1. The robot's representation in the global physics simulation.
/// 2. An internal simulator for estimating the effect of applying a control to
///    the robot from a given configuration.
///
/// Agents use the model from the global physics simulation to estimate the
/// robot's state in the simulated world.
///
/// Controllers and nonholonomic planning methods use the internal simulator to
/// compute the result of a control application.
////////////////////////////////////////////////////////////////////////////////
class DynamicsModel final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;       ///< The robot this model represents.
  btMultiBody* const m_model; ///< The robot model in the global physics engine.

  mutable InternalSimulator* m_simulator{nullptr}; ///< The internal simulator.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Create a wrapper around a physics engine model for a robot.
    /// @param[in] _r The robot being modeled.
    /// @param[in] _b The physics engine model of the robot.
    DynamicsModel(Robot* const _r, btMultiBody* const _b);

    ~DynamicsModel() noexcept;

    ///@}
    ///@name Physics Model Accessors
    ///@{

    /// Get the underlying physics model.
    btMultiBody* Get() const noexcept;

    /// Allows implicit access to underlying model pointer.
    operator btMultiBody*() const noexcept;

    ///@}
    ///@name Interface
    ///@{

    /// Get the state of the robot in the external simulation.
    Cfg GetSimulatedState() const;

    /// Test the result of applying a control to a given start configuration
    /// using the internal simulator.
    /// @param _start The starting configuration.
    /// @param _c The control to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const Control& _c, const double _dt) const;

    ///@}

};

#endif
