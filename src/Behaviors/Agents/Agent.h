#ifndef AGENT_H_
#define AGENT_H_

#include "MPProblem/MPTask.h"

class Robot;


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
    MPTask* m_task{nullptr};    ///< The current task this agent is working on.

    bool m_debug{false};               ///< Toggle debug messages.

    ///@}

  public:

    ///@name Construction
    ///@{

    Agent(Robot* const _r);

    virtual ~Agent();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the robot object which this agent belongs to.
    virtual Robot* GetRobot() const noexcept;

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
    /// Currently used for stopping the robot after reaching the end of
    /// a roadmap path.
    virtual void Halt();

    /// Set the current task for this agent. The agent will take ownership of
    /// the task and delete it when appropriate.
    /// @param _task The new current task for this agent.
    /// @TODO Make sure we handle transfering tasks from problem -> agent
    ///       correctly.
    virtual void SetCurrentTask(MPTask* const _task);

    const MPTask* GetCurrentTask() const noexcept;
    ///@}

    int m_priority = -1;               ///< The agent's priority in its group.

};

#endif
