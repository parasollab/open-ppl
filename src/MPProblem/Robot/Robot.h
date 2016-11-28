#ifndef ROBOT_H_
#define ROBOT_H_

#include <string>
#include <vector>

#include "Control.h"

class btMultiBody;
class ActiveMultiBody;
class Actuator;
class Agent;
class Boundary;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Complete representation of a robot.
///
/// @details A robot has many components, including:
///   @arg ActiveMultiBody: The robot's physical geometry.
///   @arg Agent: The robot's high-level decision-making algorithm. Determines
///               what actions the robot should take to complete its task. Used
///               only in simulations.
///   @arg ControlSet: The discrete set of controls that the robot can use, if
///                    any.
///   @arg ControlSpace: The continuous space of controls that the robot can use,
///                      if any.
///   @arg Actuators: The robot's motors/effectors. Translates control commands
///                   into generalized forces.
///   @arg BulletModel: Simulation model of the robot. Represents the robot in
///                     the bullet world.
///
/// @TODO Think about const-correctness for this object. I've left it out for
///       now because most of the functions alter the robot indirectly.
////////////////////////////////////////////////////////////////////////////////
class Robot {

  ///@name Internal State
  ///@{

  ActiveMultiBody* m_multibody{nullptr}; ///< Robot's geometric representation.

  Agent* m_agent{nullptr};               ///< Decision-making agent.
  std::vector<Actuator*> m_actuators;    ///< Actuators.
  ControlSet* m_controlSet{nullptr};     ///< Discrete control set, if any.
  ControlSpace* m_controlSpace{nullptr}; ///< Continuous control space, if any.

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
    ///@name Control Accessors
    ///@{
    /// Access the robot's control structures. The robot will take ownership of
    /// these and delete them when necessary.

    ControlSet* GetControlSet();
    void SetControlSet(ControlSet* const _c);

    ControlSpace* GetControlSpace();
    void SetControlSpace(ControlSpace* const _c);

    ///@}
    ///@name Actuator Accessors
    ///@{
    /// Access the robot's actuators. These are set during input file parsing
    /// and cannot be changed otherwise.

    Actuator* GetActuator(const size_t _i);
    const std::vector<Actuator*>& GetActuators();

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
