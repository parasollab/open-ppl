#ifndef DYNAMICS_MODEL_H_
#define DYNAMICS_MODEL_H_

#include <functional>
#include <vector>

#include "Control.h"

class BulletEngine;
class btMultiBody;
class btVector3;
class Cfg;
class Control;
class MicroSimulator;
class MultiBody;
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

  mutable MicroSimulator* m_simulator{nullptr}; ///< The internal simulator.

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

    ///@}
    ///@name Interface
    ///@{

    /// Get the state of the robot in the external simulation.
    Cfg GetSimulatedState() const;

    /// Set the state of the robot in the external simulation.
    /// @param _c The new state of the simulated robot.
    void SetSimulatedState(const Cfg& _c);

    /// Get the forces and torques on the robot in the external simulation.
    std::vector<double> GetSimulatedForces() const;

    /// Test the result of applying a control to a given start configuration
    /// using the internal simulator.
    /// @param _start The starting configuration.
    /// @param _c The control to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const Control& _c, const double _dt) const;

    /// Convert a generalized force from the robot's local frame to the world
    /// frame.
    /// @param _force The generalized force to convert.
    /// @param _model The bullet model of m_robot that defines the
    ///               transformation of interest.
    void LocalToWorld(std::vector<double>& _force, btMultiBody* const _model)
        const;

    /// @overload
    /// Convert from local to world frame using m_model.
    void LocalToWorld(std::vector<double>& _force) const;

    /// Convert a generalized force from the world frame to the robot's local
    /// frame.
    /// @param _force The generalized force to convert.
    /// @param _model The bullet model of m_robot that defines the
    ///               transformation of interest.
    void WorldToLocal(std::vector<double>& _force, btMultiBody* const _model)
        const;

    /// @overload
    /// Convert from world to local frame using m_model.
    void WorldToLocal(std::vector<double>& _force) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Transform a generalized force vector to a different coordinate frame with
    /// an arbitrary function. The function will be applied separately to the
    /// linear and angular components.
    /// @param[in] _force The generalized force vector to transform.
    /// @param[in] _f The transformation function.
    void Transform(std::vector<double>& _force,
        std::function<void(btVector3&)>&& _f) const;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A micro simulator for a single robot. It tests the result of applying a
/// control to a robot in a given configuration.
///
/// @details This object outsources forward dynamics computations to a bullet
///          physics engine.
////////////////////////////////////////////////////////////////////////////////
class MicroSimulator final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;         ///< Our pmpl robot.
  BulletEngine* const m_engine; ///< The engine for this micro-simulator.
  btMultiBody* const m_model;   ///< The bullet body for our robot.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a self-simulator for a robot.
    /// @param _robot The robot to simulate.
    MicroSimulator(Robot* const _robot);

    ~MicroSimulator();

    ///@}
    ///@name Interface
    ///@{

    /// Test the result of applying a control to the robot from a designated
    /// starting configuration.
    /// @param _start The starting configuration.
    /// @param _control The control to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const Control& _control, const double _dt);

    /// Test the result of applying a set of controls to the robot from a
    /// designated starting configuration.
    /// @param _start The starting configuration.
    /// @param _controlSet The set of controls to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const ControlSet& _controlSet, const double _dt);

    ///@}

};


/// Extract the full configuration from a simulated robot.
/// @param _robot A PMPL robot.
/// @param _model A bullet model of _robot.
/// @return The configuration data of _model in its simulation.
Cfg ExtractSimulatedState(Robot* const _robot, const btMultiBody* const _model);

/// Extract the position configuration DOFs from a simulated robot.
/// @param _mb A PMPL multibody.
/// @param _model A bullet model of _mb.
/// @return The configuration data of _mb in its simulation, excluding velocity.
std::vector<double> ExtractSimulatedPosition(MultiBody* const _mb,
    const btMultiBody* const _model);

/// Configure a simulated robot.
/// @param _c The configuration to set.
/// @param _model A bullet model of _c's robot.
void ConfigureSimulatedState(const Cfg& _c, btMultiBody* const _model);

/// Configure only the position of a simulated robot.
/// @param _v The DOF values to set.
/// @param _model A bullet model the robot.
void ConfigureSimulatedPosition(const std::vector<double>& _v,
    btMultiBody* const _model);


#endif
