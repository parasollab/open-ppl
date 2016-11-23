#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>
#include <vector>

#include "MPProblem/Robot/Actuation/Actuator.h"

class btMultiBody;
class ActiveMultiBody;
class Agent;
class Boundary;
class Controller;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Complete representation of a robot.
///
/// @details A robot has many components, including:
///   @arg ActiveMultiBody: The robot's physical geometry.
///   @arg Agent: The robot's high-level decision-making algorithm. Determines
///               what actions the robot should take to complete its task.
///   @arg Controller: The robot's low-level decision-making algorithm.
///                    Determines how to execute a higher-level action and sends
///                    the appropriate commands to the actuators.
///   @arg Actuators: The robot's motors/effectors. Translates control commands
///                   into generalized forces.
///   @arg BulletModel: Simulation model of the robot. Represents the robot in
///                     the bullet world.
////////////////////////////////////////////////////////////////////////////////
class Robot {

  ///@name Internal State
  ///@{

  ActiveMultiBody* m_multibody{nullptr}; ///< Robot's geometric representation.

  Agent* m_agent{nullptr};               ///< Robot's agent.
  Controller* m_controller{nullptr};     ///< Robot's controller.
  std::vector<Actuator> m_actuators;     ///< Robot's actuators.

  btMultiBody* m_dynamicsModel{nullptr}; ///< The bullet dynamics model.

  std::string m_label;                   ///< The robot's unique label.

  ///@}

  public:

    ///@name Construction
    ///@{

    Robot();
    Robot(XMLNode& _node, Boundary* const _b);
    virtual ~Robot();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Execute a simulation step: update the percept model, have the agent make
    /// a decision, and send the resulting controls to the actuators.
    void Step();

    ///@}
    ///@name Geometry Accessors
    ///@{
    /// Access the robot's geometric representation. The robot will take
    /// ownership of its ActiveMultiBody and delete it when necessary.

    ActiveMultiBody* GetMultiBody();
    const ActiveMultiBody* GetMultiBody() const;

    ///@}
    ///@name Agent Accessors
    ///@{
    /// Access the robot's agent. The robot will take ownership of its agent and
    /// delete it when necessary.

    Agent* GetAgent();
    void SetAgent(Agent* const _a);

    ///@}
    ///@name Controller Accessors
    ///@{
    /// Access the robot's controller. The robot will take ownership of its
    /// controller and delete it when either the controller is changed or when
    /// the robot object is destroyed.

    Controller* GetController();
    void SetController(Controller* const _c);

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    Actuator& GetActuator(const size_t _i);
    std::vector<Actuator>& GetActuators();

    ///@}
    ///@name Dynamics Accessors
    ///@{
    /// Access the robot's dynamics model. This is essentially a multibody model
    /// from whatever physics engine we are using (currently bullet). The robot
    /// does NOT assume ownership over this model - instead the physics engine
    /// is responsible for its destruction.

    btMultiBody* GetDynamicsModel();
    void SetDynamicsModel(btMultiBody* const _m);

    ///@}

};

#endif
