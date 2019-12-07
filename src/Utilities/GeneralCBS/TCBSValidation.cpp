#include "TCBSValidation.h"

#include "TMPLibrary/TaskPlan.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"

#include <unordered_set>

TCBSValidation::
TCBSValidation() {}

TCBSValidation::
TCBSValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
								Validation(_library,_lowLevel), m_tmpLibrary(_tmpLibrary) {}

TCBSValidation::
~TCBSValidation() {}

bool 
TCBSValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {
	
	CBSSolution solution;
	solution.m_decomposition = _decomposition;

	auto top = _decomposition->GetMainTask();

	for(auto task : top->GetSubtasks()) {
		for(auto subtask : task->GetSubtasks()) {
			Assignment a;
			a.m_agent = nullptr;
			a.m_task = subtask;
			solution.m_taskPlans[task].push_back(a);
		}
	}

	GeneralCBSNode node(solution);
	
	_tree.push(node);

	return true;
}

bool 
TCBSValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {

	std::vector<Assignment> unallocated;
	std::unordered_set<Agent*> occupied;


	const auto& taskPlans = _node.GetSolution().m_taskPlans;
	for(auto plan : taskPlans) {
		Agent* previous = nullptr;
		//Assume tasks are sequential in the subtask list for now
		for(auto assign : plan.second) {
			if(assign.m_agent) {
				previous = assign.m_agent;
				continue;
			}
			unallocated.push_back(assign);
		
			if(previous) {
				occupied.insert(previous);
			}

			break;
		}
	}

	if(unallocated.empty())
		return true;

	std::cout << "Creating TCBS Children" << std::endl;
	CreateAllocationChildren(_node, _tree, unallocated, occupied);
	std::cout << "Created TCBS Children" << std::endl;
	return false;
}

void
TCBSValidation::
CreateAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, 
									std::vector<Assignment>& _unallocated, std::unordered_set<Agent*>& _occupied) {
	auto team = m_tmpLibrary->GetTaskPlan()->GetTeam();

	for(auto assign : _unallocated) {
		for(auto agent : team) {
			if(_occupied.count(agent))
				continue;

			GeneralCBSNode child(_node);
			auto& solution = child.GetSolutionRef();
			auto& taskPlan = solution.m_taskPlans[assign.m_task->GetParent()];

			for(auto& a : taskPlan) {
				if(a != assign) 
					continue;
				a.m_agent = agent;
				auto& assignments = solution.m_agentAssignments[a.m_agent];
				assignments.push_back(a);
				if(m_lowLevel->UpdateSolution(child,assign.m_task->GetParent()))
					_tree.push(child);
				break;
			}
		}
	}
}
