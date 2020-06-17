#ifndef DISCRETE_AGENT_ALLOCATION_H_
#define DISCRETE_AGENT_ALLOCATION_H_

#include "Behaviors/Agents/HandoffAgent.h"

struct DiscreteAgentAllocation {
	HandoffAgent* m_agent;
	size_t m_startTime;
	size_t m_endTime;
	size_t m_startLocation;
	size_t m_endLocation;

	DiscreteAgentAllocation() {}

	DiscreteAgentAllocation(HandoffAgent* _agent, size_t _st, size_t _et, size_t _sl, size_t _el) :
													m_agent(_agent), m_startTime(_st), m_endTime(_et), m_startLocation(_sl),
													m_endLocation(_el) {};
	bool operator<(DiscreteAgentAllocation& _other) const {
		return m_startTime < _other.m_startTime;
	}

};
#endif
