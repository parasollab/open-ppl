#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>
#include <vector>

#include "Control.h"

class ActiveMultiBody;
class Actuator;
class Agent;
class btMultiBody;
class Boundary;
class ControllerMethod;
class DynamicsModel;
class MPProblem;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Complete representation of a robot.
///
/// @details A robot has many components, including:
///   @arg ActiveMultiBody: The robot's physical geometry.
///   @arg Agent: The robot's high-level decision-making algorithm. Determines
///               what actions the robot should take to complete its task. Used
///               only in simulations.
///   @arg Actuators: The robot's motors/effectors. Translates control commands
///                   into generalized forces.
///   @arg ControlSet: The discrete set of controls that the robot can use, if
///                    any.
///   @arg ControlSpace: The continuous space of controls that the robot can use,
///                      if any.
///   @arg Controller: The robot's low-level controller, which determines what
///                    control should be applied to move from point to point.
///   @arg DynamicsModel: Simulation model of the robot. Represents the robot in
///                       the bullet world.
///
/// @TODO Think about const-correctness for this object. I've left it out for
///       now because most of the functions alter the robot indirectly.
////////////////////////////////////////////////////////////////////////////////
class Robot {

  ///@name Internal State
  ///@{

  MPProblem* const m_problem;              ///< The owning problem object.

  ActiveMultiBody* m_multibody{nullptr};   ///< Robot's geometric representation.

  Agent* m_agent{nullptr};                 ///< High-level decision-making agent.

  std::vector<Actuator*> m_actuators;      ///< Actuators.
  ControlSet* m_controlSet{nullptr};       ///< Discrete control set, if any.
  ControlSpace* m_controlSpace{nullptr};   ///< Continuous control space, if any.
  ControllerMethod* m_controller{nullptr}; ///< Low-level controller.

  bool m_nonholonomic{false};              ///< Is the robot nonholonomic?
  DynamicsModel* m_dynamicsModel{nullptr}; ///< The bullet dynamics model.

  /// @TODO Should these live somewhere else? Parse them from input file.
  double m_maxLinearVelocity{10};
  double m_maxAngularVelocity{1};

  std::string m_label;                     ///< The robot's unique label.

  ///@}

  public:

    ///@name Construction
    ///@{

    Robot(MPProblem* const _p, XMLNode& _node, const Boundary* const _b);

    virtual ~Robot() noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Execute a simulation step: update the percept model, have the agent make
    /// a decision, and send the resulting controls to the actuators.
    /// @param _dt The timestep length.
    void Step(const double _dt);

    /// Enable the Robot's agent to access the problem data.
    MPProblem* GetMPProblem() const noexcept;

    ///@}
    ///@name Geometry Accessors
    ///@{
    /// Access the robot's geometric representation. The robot will take
    /// ownership of its ActiveMultiBody and delete it when necessary.

    ActiveMultiBody* GetMultiBody() noexcept;
    const ActiveMultiBody* GetMultiBody() const noexcept;

    ///@}
    ///@name Agent Accessors
    ///@{
    /// Access the robot's agent. The robot will take ownership of its agent and
    /// delete it when necessary.

    Agent* GetAgent() noexcept;
    void SetAgent(Agent* const _a) noexcept;

    ///@}
    ///@name Control Accessors
    ///@{
    /// Access the robot's control structures. The robot will take ownership of
    /// these and delete them when necessary.

    ControlSet* GetControlSet() noexcept;
    void SetControlSet(ControlSet* const _c) noexcept;

    ControlSpace* GetControlSpace() noexcept;
    void SetControlSpace(ControlSpace* const _c) noexcept;

    ControllerMethod* GetController() noexcept;
    void SetController(ControllerMethod* const _c) noexcept;

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    Actuator* GetActuator(const size_t _i) noexcept;
    const std::vector<Actuator*>& GetActuators() noexcept;

    ///@}
    ///@name Dynamics Accessors
    ///@{
    /// Access the robot's dynamics model. The robot will take ownership of it
    /// and delete it when necessary.

    DynamicsModel* GetDynamicsModel() noexcept;
    void SetDynamicsModel(btMultiBody* const _m);

    ///@}
    ///@name Other
    ///@{

    /// Check if the robot is nonholonomic.
    bool IsNonholonomic() const noexcept;

    double GetMaxLinearVelocity() const noexcept;

    double GetMaxAngularVelocity() const noexcept;

    /// Get the unique label for this instance.
    const std::string& GetLabel() const noexcept;
    ///@}

};

#endif
