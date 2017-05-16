#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <iostream>
#include <vector>

#include "Control.h"
#include "Geometry/Boundaries/Range.h"

class btMultiBody;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Models an actuator on a robot, such as an engine or motor.
///
/// @details Actuators model the effect of applying a control to the robot. They
///          accept a control and generate a generalized force on one or more of
///          the robot's DOFs. There are limits on the maximum forward/backward
///          force that can be applied to each DOF, and an additional limit on
///          the maximum total force.
///
///          Supported cases:
///          @arg Affects single DOF (done).
///          @arg Affects multiple DOF independently (done).
///          @arg Affects multiple DOF with dependence relation, i.e. the force
///               exerted on DOF i must be twice the force on DOF j (TODO).
///
///          The actuator is also able to apply a control to the simulated robot
///          with the Execute method.
///
/// @warning The max force doesn't distinguish between forces and torques: it is
///          a maximum generalized force. It's unlikely that this object will
///          make much sense if it controls both force DOFs (i.e., translation
///          or prismatic joint extension) and torque DOFs (i.e., rotation and
///          revolute/spherical joints).
////////////////////////////////////////////////////////////////////////////////
class Actuator final {

  protected:

    ///@name Local Types
    ///@{

    /// The types of dynamics supported. Force-based actuators exert forces on a
    /// robot, while velocity-based actuators simply set the velocity.
    enum DynamicsType {Force, Velocity};

    ///@}
    ///@name Internal State
    ///@{

    Robot* const m_robot;                ///< The robot that this is attached to.

    std::vector<bool> m_mask;            ///< Which DOFs can this affect?
    std::vector<Range<double>> m_limits; ///< Maximum backward/forward force.
    double m_maxForce;                   ///< The largest producable force.

    DynamicsType m_type{Force};          ///< The dynamics type.

    ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct an empty actuator for a given robot.
    /// @param _r The owning robot.
    Actuator(Robot* const _r);

    /// Construct an actuator for a given robot from an XML input.
    /// @param _r The owning robot.
    /// @param _node The XML input node.
    Actuator(Robot* const _r, XMLNode& _node);

    ///@}
    ///@name Actuator Properties
    ///@{

    /// Set the minimum and maximum generalized forces that this actuator can
    /// apply to any DOF.
    /// @param[in] _min The minimum generalized forces.
    /// @param[in] _max The maximum generalized forces.
    void SetLimits(const std::vector<double>& _min,
        const std::vector<double>& _max);

    /// Set the maximum total generalized force that this actuator can apply.
    /// @param[in] _total The greatest total generalized force that can be
    ///                   applied.
    void SetMaxForce(const double _total);

    ///@}
    ///@name Planning Interface
    ///@{

    /// Get the control mask for this actuator, which indicates the DOFs that it
    /// can affect.
    /// @return A vector with one boolean for each DOF, where true indicates that
    ///         the actuator can affect that DOF.
    std::vector<bool> ControlMask() const;

    /// Compute the force generated by driving the actuator at some fraction of
    /// its total power.
    /// @param[in] _s The control signal to apply.
    /// @return The force generated by driving this with control signal _s.
    std::vector<double> ComputeForce(const Control::Signal& _s) const;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Apply a control signal to the robot's bullet model.
    /// @param _s The control signal to apply.
    void Execute(const Control::Signal& _s) const;

    /// Apply a control signal to a bullet model of the owning robot.
    /// @param _s The control signal to apply.
    /// @param _model The bullet model of m_robot to control.
    void Execute(const Control::Signal& _s, btMultiBody* const _model) const;

    ///@}
    ///@name Debug
    ///@{

    /// Print an actuator's state to an ostream.
    friend std::ostream& operator<<(std::ostream&, const Actuator&);

    ///@}
};

#endif
