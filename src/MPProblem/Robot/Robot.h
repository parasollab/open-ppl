#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

#include "Control.h"

class ActiveMultiBody;
class Actuator;
class Agent;
class btMultiBody;
class Boundary;
class ControllerMethod;
class CSpaceBoundingBox;
class DynamicsModel;
class HardwareInterface;
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

  std::string m_label;                     ///< The robot's unique label.

  ActiveMultiBody* m_multibody{nullptr};   ///< Robot's geometric representation.

  bool m_virtual{false};                   ///< Is this an imaginary robot?

  std::string m_agentLabel;                ///< Agent type label.
  Agent* m_agent{nullptr};                 ///< High-level decision-making agent.

  std::unordered_map<std::string, Actuator*> m_actuators; ///< Actuators.

  ControllerMethod* m_controller{nullptr}; ///< Low-level controller.

  bool m_nonholonomic{false};              ///< Is the robot nonholonomic?
  bool m_carlike{false};                   ///< Is the robot car-like?
  DynamicsModel* m_dynamicsModel{nullptr}; ///< The bullet dynamics model.

  double m_maxLinearVelocity{10};          ///< Max linear velocity.
  double m_maxAngularVelocity{1};          ///< Max angular velocity.

  CSpaceBoundingBox* m_cspace{nullptr};    ///< The robot's configuration space.
  CSpaceBoundingBox* m_vspace{nullptr};    ///< The robot's velocity space.

  /// Interfaces the robot's hardware, mapped by label.
  std::unordered_map<std::string, HardwareInterface*> m_hardware;

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a robot from an XML node.
    /// @param[in] _p The owning MPProblem.
    /// @param[in] _node The XML node to parse.
    Robot(MPProblem* const _p, XMLNode& _node);

    /// Construct a robot from a multibody.
    /// @param[in] _p The owning MPProblem.
    /// @param[in] _label The unique label for this robot.
    Robot(MPProblem* const _p, ActiveMultiBody* const _mb,
        const std::string& _label);

    virtual ~Robot() noexcept;

    ///@}

  protected:

    ///@name I/O
    ///@{

    /// Parse an XML robot file.
    /// @param[in] _filename The file name.
    void ReadXMLFile(const std::string& _filename);

    /// Parse a multibody file describing this robot.
    /// @param[in] _filename The file name.
    void ReadMultibodyFile(const std::string& _filename);

    /// Compute the configuration and velocity spaces for this robot.
    void InitializePlanningSpaces();

    ///@}

  public:

    ///@name Planning Interface
    ///@{

    /// Get the configuration space boundary for this robot.
    const CSpaceBoundingBox* GetCSpace() const noexcept;

    /// Get the velocity space boundary for this robot.
    const CSpaceBoundingBox* GetVSpace() const noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Execute a simulation step: update the percept model, have the agent make
    /// a decision, and send the resulting controls to the actuators.
    /// @param[in] _dt The timestep length.
    void Step(const double _dt);

    /// Get the MPProblem which owns this robot.
    MPProblem* GetMPProblem() const noexcept;

    /// Align the multibody model to the robot's current simulated state.
    void SynchronizeModels() noexcept;

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

    ControllerMethod* GetController() noexcept;
    void SetController(ControllerMethod* const _c) noexcept;

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    Actuator* GetActuator(const std::string& _label) noexcept;
    const std::unordered_map<std::string, Actuator*>& GetActuators() noexcept;

    ///@}
    ///@name Dynamics Accessors
    ///@{
    /// Access the robot's dynamics model. The robot will take ownership of it
    /// and delete it when necessary.

    DynamicsModel* GetDynamicsModel() noexcept;
    void SetDynamicsModel(btMultiBody* const _m);

    ///@}
    ///@name Hardware Interface
    ///@{
    /// Access the interface to the hardware robot (if any).

    HardwareInterface* GetHardwareInterface(const std::string& _label) const
        noexcept;
    void SetHardwareInterface(const std::string& _label,
        HardwareInterface* const _i) noexcept;

    /// Get the minimum time between commands sent to the hardware, in seconds.
    double GetHardwareTime() const noexcept;

    ///@}
    ///@name Other Properties
    ///@{

    /// Check if this is a 'virtual' robot. These do not represent physical
    /// robots, and will be ignored by other robots in collision detection.
    /// Virtual robots will not appear at all in any simulations.
    bool IsVirtual() const noexcept;

    /// Check if the robot is nonholonomic.
    bool IsNonholonomic() const noexcept;

    /// Check if the robot is car-like.
    bool IsCarlike() const noexcept;

    /// Get the maximum translational velocity for this robot.
    double GetMaxLinearVelocity() const noexcept;

    /// Get the maximum angular velocity for this robot.
    double GetMaxAngularVelocity() const noexcept;

    /// Get the unique label for this robot.
    const std::string& GetLabel() const noexcept;

    /// Change the robot's unique label.
    void SetLabel(const std::string&);

    ///@}
    ///@name Debug
    ///@{

    friend std::ostream& operator<<(std::ostream&, const Robot&);

    ///@}

};

#endif
