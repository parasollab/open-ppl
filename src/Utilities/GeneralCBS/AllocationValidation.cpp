#include "AllocationValidation.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "Utilities/GeneralCBS/TMPLowLevelSearch.h"
#include "Utilities/MPUtils.h"

AllocationValidation::
AllocationValidation() {}	

AllocationValidation::
AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
		Validation(_library, _lowLevel), 
		m_tmpLibrary(_tmpLibrary) {}	

AllocationValidation::
~AllocationValidation() {}

bool
AllocationValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {

	CBSSolution solution;
	solution.m_decomposition = _decomposition;
	GeneralCBSNode node(solution);


	//temp for RA-L problem structure

	//add robot initial locations as constraints
	std::vector<AllocationConstraint> initialLocations;

	for(auto agent : m_tmpLibrary->GetTaskPlan()->GetTeam()) {
		Cfg location = agent->GetRobot()->GetSimulationModel()->GetState();

		auto dummy = m_tmpLibrary->GetTaskPlan()->GetCapabilityAgent(agent->GetCapability());
		location.SetRobot(dummy->GetRobot());

		AllocationConstraint constraint(nullptr,location,location,0,0,nullptr);

		for(auto& semanticTask : _decomposition->GetSimpleTasks()) {
			constraint.m_task = semanticTask;
			node.AddAllocationConstraint(constraint, agent);
		}
	}

	for(auto& semanticTask : _decomposition->GetSimpleTasks()) {
		if(!m_lowLevel->UpdateSolution(node, semanticTask))
			return false;	
	}

	_tree.push(node);
	return true;
}

bool 
AllocationValidation::
ValidatePlan(GeneralCBSNode& _node, GeneralCBSTree& _tree) {
	auto constraints = FindAllocationConflict(_node);

	if(constraints.first.m_startTime != MAX_DBL) {
		AddAllocationChildren(_node, _tree, constraints);
		return false;
	}

	return true;
}

std::pair<AllocationConstraint,AllocationConstraint>
AllocationValidation::
FindAllocationConflict(GeneralCBSNode& _node) {
	Assignment conf1;
	Assignment conf2;

	conf1.m_execStartTime = MAX_DBL;
	Agent* confAgent{nullptr};

	auto solution = _node.GetSolution();
	//step through all agent assignments and see if any overlap, 
	//if so, look at the assignment's task's parent to get conflicting task
	//TODO::find a more clever way to do this that simultaneosuly iteratively looks at agent plans
	for(auto& agentAssigns : solution.m_agentAssignments) {
		auto& agent = agentAssigns.first;
		auto& assignments = agentAssigns.second;

		if(assignments.size() == 0)
			continue;

		for(size_t i = 0; i < assignments.size()-1; i++) {
			auto& a1 = assignments[i];
			auto& a2 = assignments[i+1];

			if(a1.m_execStartTime == a2.m_execStartTime or
					a1.m_execEndTime > a2.m_execStartTime or 
					!CanReach(a1,a2)) {
				if(a1.m_execStartTime < conf1.m_execStartTime) {
					conf1 = a1;
					conf2 = a2;
					confAgent = agent;
				}
				break;
			}
		}
	}

	if(!confAgent){
		return std::make_pair(AllocationConstraint(),AllocationConstraint());
	}

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph("MTGraph").get());
	auto r = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(conf1.m_agent));

	AllocationConstraint one(confAgent,
														r->GetVertex(conf2.m_execPath.front()),
														r->GetVertex(conf2.m_execPath.back()),
														conf2.m_execStartTime,
														conf2.m_execEndTime,
														conf1.m_task->GetParent());
	AllocationConstraint two(confAgent,
														r->GetVertex(conf1.m_execPath.front()),
														r->GetVertex(conf1.m_execPath.back()),
														conf1.m_execStartTime,
														conf1.m_execEndTime,
														conf2.m_task->GetParent());

	return std::make_pair(one,two);
}

void
AllocationValidation::
AddAllocationChildren(GeneralCBSNode& _node, GeneralCBSTree& _tree, AllocationConstraintPair _constraints) {

	GeneralCBSNode one(_node);
	one.AddAllocationConstraint(_constraints.first,_constraints.first.m_agent);
	if(m_lowLevel->UpdateSolution(one, _constraints.first.m_task))
		_tree.push(one);
 
	GeneralCBSNode two(_node);
	two.AddAllocationConstraint(_constraints.second,_constraints.second.m_agent);
	if(m_lowLevel->UpdateSolution(two, _constraints.second.m_task))
		_tree.push(two);
}

bool
AllocationValidation::
CanReach(Assignment& _a1, Assignment& _a2) {

	size_t startVID = _a1.m_execPath.back();
	size_t goalVID = _a2.m_execPath.front();

	//TODO::Split motion plan function in TMPLowLevel to take in these VIDs since the cfg fecthing is repeated	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph("MTGraph").get());
	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(_a1.m_agent));

	auto tmp = static_cast<TMPLowLevelSearch*>(m_lowLevel);

	auto plan = tmp->MotionPlan(roadmap->GetVertex(startVID),roadmap->GetVertex(goalVID));

	if(plan.first + _a1.m_execEndTime > _a2.m_execStartTime) {
		return false;
	}

	_a2.m_setupPath = plan.second;

	return true;
}
