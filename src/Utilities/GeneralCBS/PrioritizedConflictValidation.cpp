#include "PrioritizedConflictValidation.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "Utilities/GeneralCBS/TMPLowLevelSearch.h"
#include "Utilities/MPUtils.h"
/*------------------------------------ Construction ------------------------------------*/

PrioritizedConflictValidation::
PrioritizedConflictValidation(MPLibrary* _library, LowLevelSearch* _lowLevelSearch, 
									TMPLibrary* _tmpLibrary, FindAllConflictsFunction _findAll) :
						Validation(_library, _lowLevelSearch, _tmpLibrary), m_findAll(_findAll) {}

PrioritizedConflictValidation::
~PrioritizedConflictValidation() {}

/*-------------------------------------- Interface ------------------------------------*/

bool 
PrioritizedConflictValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	
	m_findAll(_node);

	std::vector<GeneralCBSNode> semiCardinal;
	std::vector<GeneralCBSNode> nonCardinal;

	for(auto& conflict : _node.GetAllocationConflicts()) {

			auto constraints = GetAllocationConstraints(conflict);

			auto children = CreateAllocationChildren(_node, constraints);

			size_t cardinality = 0;
			for(auto child : children) {
				if(child.GetCost() > _node.GetCost()) 
					cardinality++;
			}

			// Cardinal Conflict
			if(cardinality == children.size()) {
				for(auto child : children) {
					_tree.push(child);
				}
				return false;
			}
	
			// Semi-Cardinal Conflict
			if(cardinality > 0 and semiCardinal.empty()) {
				semiCardinal = children;
				continue;
			}

			// Non-Cardinal Conflict
			if(nonCardinal.empty())
				nonCardinal = children;
	}

	if(!semiCardinal.empty()) {
		for(auto child : semiCardinal) {
			_tree.push(child);
		}
		return false;
	}
		

	if(!nonCardinal.empty()) {
		for(auto child : nonCardinal) {
			_tree.push(child);
		}
		return false;
	}

	//TODO::Add Motion Conflict Checking Here

	return true;
}

/*---------------------------------- Helper Functions ---------------------------------*/

std::pair<AllocationConstraint,AllocationConstraint>
PrioritizedConflictValidation::
GetAllocationConstraints(std::pair<Assignment,Assignment>& _conflict) {
	auto conf1 = _conflict.first;
	auto conf2 = _conflict.second;

	auto confAgent = conf1.m_agent;

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph("MTGraph").get());
	auto r = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(conf1.m_agent));

	Cfg startCfg = r->GetVertex(conf2.m_execPath.front());
	startCfg.SetRobot(confAgent->GetRobot());
	Cfg endCfg = r->GetVertex(conf2.m_execPath.back());
	endCfg.SetRobot(confAgent->GetRobot());

	AllocationConstraint one(confAgent,
														//r->GetVertex(conf2.m_execPath.front()),
														//r->GetVertex(conf2.m_execPath.back()),
														startCfg,
														endCfg,
														conf2.m_execStartTime,
														conf2.m_execEndTime,
														conf1.m_task->GetParent());

	startCfg = r->GetVertex(conf1.m_execPath.front());
	startCfg.SetRobot(confAgent->GetRobot());
	endCfg = r->GetVertex(conf1.m_execPath.back());
	endCfg.SetRobot(confAgent->GetRobot());

	AllocationConstraint two(confAgent,
														//r->GetVertex(conf1.m_execPath.front()),
														//r->GetVertex(conf1.m_execPath.back()),
														startCfg,
														endCfg,
														conf1.m_execStartTime,
														conf1.m_execEndTime,
														conf2.m_task->GetParent());

	return std::make_pair(one,two);
}

std::vector<GeneralCBSNode>
PrioritizedConflictValidation::
CreateAllocationChildren(GeneralCBSNode& _node, 
			std::pair<AllocationConstraint,AllocationConstraint> _constraints) {
	
	if(!_constraints.first.m_task or !_constraints.second.m_task) {
		throw RunTimeException(WHERE, "Houston we have a problem.");
	}

	std::vector<GeneralCBSNode> children;

	GeneralCBSNode one(_node);
	one.AddAllocationConstraint(_constraints.first,_constraints.first.m_agent);
	if(m_lowLevel->UpdateSolution(one, _constraints.first.m_task))
		children.push_back(one);
 
	GeneralCBSNode two(_node);
	two.AddAllocationConstraint(_constraints.second,_constraints.second.m_agent);
	if(m_lowLevel->UpdateSolution(two, _constraints.second.m_task))
		children.push_back(two);

	return children;
}
