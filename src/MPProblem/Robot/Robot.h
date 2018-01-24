#ifndef ROBOT_H_
#define ROBOT_H_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "Control.h"

class Actuator;
class Agent;
class btMultiBody;
class Boundary;
class ControllerMethod;
class CSpaceBoundingBox;
class DynamicsModel;
class HardwareInterface;
class MPProblem;
class MultiBody;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Complete representation of a robot.
///
/// @details A robot has many components, including:
///   @arg MultiBody: The robot's physical geometry.
///   @arg Agent: The robot's high-level decision-making algorithm. Determines
///               what actions the robot should take to complete its task. Used
///               only in simulations.
///   @arg Actuators: The robot's motors/effectors. Translates control commands
///                   into generalized forces.
///   @arg Controller: The robot's low-level controller, which determines what
///                    control should be applied to move from point to point.
///   @arg DynamicsModel: Simulation model of the robot. Represents the robot in
///                       the bullet world.
////////////////////////////////////////////////////////////////////////////////
class Robot final {

  ///@name Internal State
  ///@{

  MPProblem* m_problem{nullptr};              ///< The owning problem object.

  std::string m_label;                     ///< The robot's unique label.

  bool m_virtual{false};                   ///< Is this an imaginary robot?

  std::unique_ptr<MultiBody> m_multibody;  ///< Robot's geometric representation.

  /// Actuators.
  std::unordered_map<std::string, std::unique_ptr<Actuator>> m_actuators;

  std::unique_ptr<ControllerMethod> m_controller; ///< Low-level controller.

  bool m_nonholonomic{false};              ///< Is the robot nonholonomic?
  bool m_carlike{false};                   ///< Is the robot car-like?
  std::unique_ptr<DynamicsModel> m_dynamicsModel; ///< The bullet dynamics model.

  double m_maxLinearVelocity{10};          ///< Max linear velocity.
  double m_maxAngularVelocity{1};          ///< Max angular velocity.

  std::unique_ptr<CSpaceBoundingBox> m_cspace; ///< The robot's c-space.
  std::unique_ptr<CSpaceBoundingBox> m_vspace; ///< The robot's velocity space.

  std::unique_ptr<Agent> m_agent;          ///< High-level decision-making agent.

  /// Interfaces the robot's hardware, mapped by label.
  std::unordered_map<std::string, std::unique_ptr<HardwareInterface>> m_hardware;

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
    Robot(MPProblem* const _p, std::unique_ptr<MultiBody>&& _mb,
        const std::string& _label);

    /// Copy a robot to another MPProblem.
    /// @param _p The destination MPProblem.
    /// @param _r The source robot to copy.
    Robot(MPProblem* const _p, const Robot& _r);

    ~Robot() noexcept;

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Regular copy/move is not allowed because we require a permanent
    /// MPProblem for the robot to belong to.
    /// Assignment is disabled because we should never need to re-assign entire
    /// robots. Destruct the old one and create a new one instead.

    Robot(const Robot&) = delete;
    Robot(Robot&&);

    Robot& operator=(const Robot&) = delete;
    Robot& operator=(Robot&&);

    ///@}

  protected:

    ///@name I/O
    ///@{

    /// Parse an XML robot file.
    /// @param[in] _filename The file name.
    void ReadXMLFile(const std::string& _filename);

    /// Parse multibody information from robot's XML file.
    /// @param[in] _node The XML node to parse
    void ReadMultiBodyXML(XMLNode& _node);

    /// Parse a multibody file describing this robot.
    /// @param[in] _filename The file name.
    void ReadMultibodyFile(const std::string& _filename);

    ///@}

  public:

    ///@name Planning Interface
    ///@{

    /// Compute the configuration and velocity spaces for this robot.
    void InitializePlanningSpaces();

    /// Get the configuration space boundary for this robot.
    const CSpaceBoundingBox* GetCSpace() const noexcept;

    /// Get the velocity space boundary for this robot.
    const CSpaceBoundingBox* GetVSpace() const noexcept;

    /// Get the owning MPProblem.
    MPProblem* GetMPProblem() const noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Execute a simulation step: update the percept model, have the agent make
    /// a decision, and send the resulting controls to the actuators.
    /// @param[in] _dt The timestep length.
    void Step(const double _dt);

    /// Align the multibody model to the robot's current simulated state.
    void SynchronizeModels() noexcept;

    ///@}
    ///@name Geometry Accessors
    ///@{
    /// Access the robot's geometric representation. The robot will take
    /// ownership of its MultiBody and delete it when necessary.

    MultiBody* GetMultiBody() noexcept;
    const MultiBody* GetMultiBody() const noexcept;

    ///@}
    ///@name Agent Accessors
    ///@{
    /// Access the robot's agent. The robot will take ownership of its agent and
    /// delete it when necessary.

    Agent* GetAgent() noexcept;
    void SetAgent(std::unique_ptr<Agent>&& _a) noexcept;

    ///@}
    ///@name Control Accessors
    ///@{
    /// Access the robot's control structures. The robot will take ownership of
    /// these and delete them when necessary.

    ControllerMethod* GetController() noexcept;
    void SetController(std::unique_ptr<ControllerMethod>&& _c) noexcept;

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    Actuator* GetActuator(const std::string& _label) noexcept;
    const std::unordered_map<std::string, std::unique_ptr<Actuator>>&
        GetActuators() const noexcept;

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
        std::unique_ptr<HardwareInterface>&& _i) noexcept;

    /// Get the minimum time between commands sent to the hardware, in seconds.
    double GetHardwareTime() const noexcept;

    ///@}
    ///@name Other Properties
    ///@{

    /// Check if this is a 'virtual' robot. These do not represent physical
    /// robots, and will be ignored by other robots in collision detection.
    /// Virtual robots will not appear in simulations.
    bool IsVirtual() const noexcept;

    /// Set the robot's virtual flag.
    /// @param _v The new value for the virtual flag.
    void SetVirtual(const bool _v) noexcept;

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

    ///@}

};

/*----------------------------------- Debug ----------------------------------*/

std::ostream& operator<<(std::ostream&, const Robot&);

/*----------------------------------------------------------------------------*/

#endif
