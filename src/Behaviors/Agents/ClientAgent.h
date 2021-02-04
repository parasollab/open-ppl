#ifndef _PPL_CLIENT_AGENT_
#define _PPL_CLIENT_AGENT_

/*--------------------------------------------------------------------*/
/// Simple wrapper around the base agent that receives commands from 
/// an agent publisher and executes them in the simulator.
/*--------------------------------------------------------------------*/

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <queue>
#include <vector>

#include "Agent.h"

class ClientAgent : public Agent {
	public:
		///@name Construction
		///@{

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    ClientAgent(Robot* const _r);

    /// Create an agent for a robot.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    ClientAgent(Robot* const _r, XMLNode& _node);

    ClientAgent(Robot* const _r, const ClientAgent& _a);

		~ClientAgent();

		std::unique_ptr<Agent> Clone(Robot* const _r) const override;

		///@}
		///@name Simulator Interface
		///@{

    /// Set up the agent before running. Anything that needs to be done only once
    /// on first starting should go here.
    virtual void Initialize() override;

    /// Decide what to do on each time step in the simulation. The agent should
    /// implement its decision by sending commands to the robot's controller.
    /// @param _dt The timestep length.
    virtual void Step(const double _dt) override;

    /// Tear down the agent. Release any resources and reset the object to it's
    /// pre-initialize state.
    virtual void Uninitialize() override;

		///@}

	private:

		///@name Communication Interface
		///@{

		std::pair<size_t,ControlSet> ListenForControls();

		///@}
		///@name Internal State
		///@{

		std::queue<std::pair<size_t,ControlSet>> m_controlQueue;

		mutable std::atomic<bool> m_running;

		std::thread m_thread;

    mutable std::mutex m_lock;   

		std::string m_controlChannel;
		///@}
		
};

#endif

