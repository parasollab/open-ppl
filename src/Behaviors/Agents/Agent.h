#ifndef AGENT_H_
#define AGENT_H_

#include <memory>

class MPTask;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// The decision-making faculties of a robot.
///
/// @details Agents are used with simulated robots. On each time step of the
///          simulation, the agent decides what the robot will do and sends an
///          appropriate command to the controller.
////////////////////////////////////////////////////////////////////////////////
class Agent {

  protected:

    ///@name Internal State
    ///@{

    Robot* const m_robot;              ///< The robot that this agent controls.
    mutable bool m_initialized{false}; ///< Is the agent initialized?
    MPTask* m_task{nullptr};           ///< The current task this agent is working on.

    bool m_debug{false};               ///< Toggle debug messages.

    ///@}

  public:

    ///@name Construction
    ///@{

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    Agent(Robot* const _r);

    /// Copy an agent for another robot.
    /// @param _r The destination robot.
    /// @param _a The agent to copy.
    Agent(Robot* const _r, const Agent& _a);

    /// Create a dynamically-allocated agent from an XML node.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    /// @return An agent of the type specified by _node.
    static std::unique_ptr<Agent> Factory(Robot* const _r, XMLNode& _node);

    /// Create a copy of this agent for another robot. This is provided so that
    /// we can copy an agent without knowing its type.
    virtual std::unique_ptr<Agent> Clone(Robot* const _r) const = 0;

    virtual ~Agent();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot object which this agent belongs to.
    virtual Robot* GetRobot() const noexcept;

    ///@}
    ///@name Disabled Functions
    ///@{
    /// Regular copy/move is disabled because each agent must be created for a
    /// specific robot object.

    Agent(const Agent&) = delete;
    Agent(Agent&&) = delete;

    Agent& operator=(const Agent&) = delete;
    Agent& operator=(Agent&&) = delete;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Set up the agent before running. Anything that needs to be done only once
    /// on first starting should go here.
    virtual void Initialize() = 0;

    /// Decide what to do on each time step in the simulation. The agent should
    /// implement its decision by sending commands to the robot's controller.
    /// @param _dt The timestep length.
    virtual void Step(const double _dt) = 0;

    /// Tear down the agent. Release any resources and reset the object to it's
    /// pre-initialize state.
    virtual void Uninitialize() = 0;

    /// Stop the robot in simulation (places 0s in all 6 velocity dofs).
    /// @WARNING Arbitrarily setting the velocity does not respect the robot's
    ///          dynamics. It is OK for debugging and freezing a scenario upon
    ///          completion, but it is not physically realistic.
    virtual void Halt();

    /// Set the current task for this agent. The agent will take ownership of
    /// the task and delete it when appropriate.
    /// @param _task The new current task for this agent.
    /// @TODO Make sure we handle transfering tasks from problem -> agent
    ///       correctly.
    virtual void SetCurrentTask(MPTask* const _task);

    /// Get the current task that the agent is working on.
    const MPTask* GetCurrentTask() const noexcept;

    ///@}

    /// TODO move
    int m_priority = -1;               ///< The agent's priority in its group.

};

#endif
