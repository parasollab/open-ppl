#include "GeneralCBS.h"

#include "MPProblem/Robot/Robot.h"
#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"

GeneralCBSNode::
GeneralCBSNode(const CBSSolution _solution) {
	m_solution = _solution;
}

GeneralCBSNode::
GeneralCBSNode(const CBSSolution _solution, const CBSSolution _postAssignment) {
	m_solution = _solution;
	m_postAssignment = _postAssignment;
}

GeneralCBSNode::
GeneralCBSNode(const GeneralCBSNode& _parent) {
	m_solution = _parent.m_solution;
	m_motionConstraints = _parent.m_motionConstraints;
	m_allocationConstraints = _parent.m_allocationConstraints;
	m_cost = _parent.m_cost;
	m_postAssignment = _parent.m_postAssignment;
}

bool 
GeneralCBSNode::
operator>(const GeneralCBSNode& _other) const noexcept {

	if(GetUtility() == _other.GetUtility()){
		return GetCost() > _other.GetCost();
	}
		
	return GetUtility() < _other.GetUtility();
}

bool 
GeneralCBSNode::
operator<(const GeneralCBSNode& _other) const noexcept {

	if(GetUtility() == _other.GetUtility())
		return GetCost() < _other.GetCost();

	return GetUtility() > _other.GetUtility();
}

CBSSolution 
GeneralCBSNode::
GetSolution() const {
	return m_solution;
}
	
CBSSolution&
GeneralCBSNode::
GetSolutionRef() {
	return m_solution;
}
	
void 
GeneralCBSNode::
SetSolution(const CBSSolution& _solution) {
	m_solution = _solution;
}

CBSSolution&
GeneralCBSNode::
GetPostAssignmentRef() {
	return m_postAssignment;
}

void 
GeneralCBSNode::
AddMotionConstraint(MotionConstraint& _c, Agent* _agent) {
	m_motionConstraints[_c.m_task][_agent].push_back(_c);	
}

const std::vector<MotionConstraint>&
GeneralCBSNode::
GetMotionConstraints(SemanticTask* _task, Agent* _agent) {
	return m_motionConstraints[_task][_agent];
}

void 
GeneralCBSNode::
AddAllocationConstraint(AllocationConstraint& _c, Agent* _agent) {
	m_allocationConstraints[_c.m_task][_agent].push_back(_c);
}

const std::vector<AllocationConstraint>&
GeneralCBSNode::
GetAllocationConstraints(SemanticTask* _task, Agent* _agent) {
	return m_allocationConstraints[_task][_agent];
}

void 
GeneralCBSNode::
UpdateTaskPlan(SemanticTask* _task, std::vector<Assignment> _assignments) {
	//TODO::update solution from this input assignment
	
	m_solution.m_taskPlans[_task] = _assignments;

	for(auto& agentAssigns: m_solution.m_agentAssignments) {
		auto& assigns = agentAssigns.second;
		for(size_t i = 0; i < assigns.size(); i++) {
			if(assigns[i].m_task->GetParent() == _task) {
				assigns.erase(assigns.begin()+i);
				break;
			}
		}
	}

	for(auto assign : _assignments) {
		//std::vector<Assignment> temp = {assign};

		auto& agentAssigns = m_solution.m_agentAssignments[assign.m_agent];
		agentAssigns.push_back(assign);
		
		/*for(auto a : agentAssigns) {
			if(a.m_task->GetParent() != _task) {
				temp.push_back(a);
			}
		}*/

		//std::sort(temp.begin(),temp.end());
		//m_solution.m_agentAssignments[assign.m_agent] = temp;
		std::sort(agentAssigns.begin(),agentAssigns.end());
		//probably don't need in current structure
		m_solution.m_allocationMap[assign.m_task] = assign.m_agent;
	}

	//Just computing SOC for now
	double cost = 0;
	for(auto plan : m_solution.m_taskPlans) {
		auto a = plan.second.back();
		cost += a.m_execEndTime;
	}
	m_cost = cost;

}

void
GeneralCBSNode::
Debug() {
	std::cout << "Allocation Constraints" << std::endl;
	for(auto tACs : m_allocationConstraints) {
		std::cout << "Task: " << tACs.first << std::endl;
		for(auto aCs : tACs.second) {				std::cout << aCs.first->GetRobot()->GetLabel() << std::endl;
			std::cout << aCs.first->GetRobot()->GetLabel() << std::endl;
			for(auto c : aCs.second) {
				std::cout << c.m_startLocation.PrettyPrint() << std::endl;
				std::cout << c.m_endLocation.PrettyPrint() << std::endl;
				std::cout << c.m_startTime << std::endl;
				std::cout << c.m_endTime << std::endl;
			}
		}
	}
	std::cout << "Motion Constraints" << std::endl;
	for(auto tACs : m_motionConstraints) {
		std::cout << "Task: " << tACs.first << std::endl;
		for(auto aCs : tACs.second) {
			std::cout << aCs.first->GetRobot()->GetLabel() << std::endl;
			for(auto c : aCs.second) {
				std::cout << c.m_conflictCfg.PrettyPrint() << std::endl;
				std::cout << c.m_timestep << std::endl;
			}
		}
	}
	std::cout << "Task Plans" << std::endl;
	for(auto tp : m_solution.m_taskPlans) {
		std::cout << std::endl << "\tTask: " << tp.first << std::endl;
		for(auto a : tp.second) {
			if(!a.m_agent)
				continue;
			std::cout << "\t\t" << a.m_agent->GetRobot()->GetLabel() << std::endl;
			std::cout << "\t\t\t" << a.m_setupStartTime << "-------->" 
								<< a.m_execStartTime << "---->" << a.m_execEndTime << std::endl;
		}
	}	
}

void
GeneralCBSNode::
SetCost(double _cost) {
	m_cost = _cost;
}

double
GeneralCBSNode::
GetCost() const {
	//return m_cost;
	//TODO::right now, rest of code default computes the sum-of-cost - needs to be paramaterized
	//temp makespan override:
	double makespan = 0;
	for(auto& tp : m_solution.m_taskPlans) {
		if(tp.second.back().m_execEndTime > makespan)
			makespan = tp.second.back().m_execEndTime;
	}
	return makespan;
}
	
void 
GeneralCBSNode::
SetUtility(double _utility) {
	m_utility = _utility;
}

double 
GeneralCBSNode::
GetUtility() const {
	return m_utility;
}

size_t 
GeneralCBSNode::
GetAllocationConflictCount() {
	return m_allocationConflictCount; 
}
	
size_t 
GeneralCBSNode::
GetMotionConflictCount() {
	return m_motionConflictCount;
}

void 
GeneralCBSNode::
SetAllocationConflictCount(size_t _c) {
	m_allocationConflictCount = _c;
}
	
void
GeneralCBSNode::
SetMotionConflictCount(size_t _c) {
	m_motionConflictCount = _c;
}

/*-------------------------- Conflict Based Search -------------------------*/

CBSSolution
ConflictBasedSearch(Decomposition* _decomposition, InitialPlanFunction _initial, 
										ValidationFunction _validation, size_t _numIterations, bool _debug) {

	GeneralCBSTree tree;

	size_t counter = 0;

	if(!_initial(_decomposition, tree))
		return CBSSolution();

	GeneralCBSNode current;

	do {

		current = tree.top();
		tree.pop();

		counter++;
	
		if(_debug) {
			current.Debug();
			std::cout << "Total Nodes Explored: " << counter << std::endl;
			std::cout << "Current Tree Size: " << tree.size() << std::endl;
		}

		if(!_validation(current, tree))
			continue;

		Simulation::GetStatClass()->SetStat("TotalNodes", counter + tree.size());
		Simulation::GetStatClass()->SetStat("NodesExplored", counter);
		Simulation::GetStatClass()->SetStat("SOC", current.GetCost());
		return current.GetSolution();

	} while(!tree.empty() and counter < _numIterations);

	Simulation::GetStatClass()->SetStat("TotalNodes", counter + tree.size());
	Simulation::GetStatClass()->SetStat("NodesExplored", counter);
	Simulation::GetStatClass()->SetStat("SOC", current.GetCost());
	return CBSSolution();
	
}
	
