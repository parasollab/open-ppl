#include "AllocationValidation.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "Utilities/GeneralCBS/TMPLowLevelSearch.h"
#include "Utilities/MPUtils.h"

AllocationValidation::
AllocationValidation() {}	

AllocationValidation::
AllocationValidation(MPLibrary* _library, LowLevelSearch* _lowLevel, TMPLibrary* _tmpLibrary) : 
		Validation(_library, _lowLevel, _tmpLibrary) { }	

AllocationValidation::
~AllocationValidation() {}

bool
AllocationValidation::
InitialPlan(Decomposition* _decomposition, GeneralCBSTree& _tree) {

	CBSSolution solution;
	solution.m_decomposition = _decomposition;

	CBSSolution postAssignment = this->CreatePostAssignment();

	GeneralCBSNode node(solution,postAssignment);


	//temp for RA-L problem structure

	//add robot initial locations as constraints
	std::vector<AllocationConstraint> initialLocations;

	for(auto agent : m_tmpLibrary->GetTaskPlan()->GetTeam()) {
		Cfg location = agent->GetRobot()->GetSimulationModel()->GetState();

		//auto dummy = m_tmpLibrary->GetTaskPlan()->GetCapabilityAgent(agent->GetCapability());
		//location.SetRobot(dummy->GetRobot());

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

	//PatchPaths(_node);
	return true;
}

std::pair<AllocationConstraint,AllocationConstraint>
AllocationValidation::
FindAllocationConflict(GeneralCBSNode& _node) {
	Assignment conf1;
	Assignment conf2;

	conf1.m_execStartTime = MAX_DBL;
	Agent* confAgent{nullptr};

	auto& solution = _node.GetSolutionRef();
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
					!CanReach(a1,a2,_node)) {
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
CanReach(Assignment& _a1, Assignment& _a2, GeneralCBSNode& _node) {

	size_t startVID = _a1.m_execPath.back();
	size_t goalVID = _a2.m_execPath.front();

	//TODO::Split motion plan function in TMPLowLevel to take in these VIDs since the cfg fecthing is repeated	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph("MTGraph").get());
	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(_a1.m_agent));

	auto tmp = static_cast<TMPLowLevelSearch*>(m_lowLevel);

	auto team = m_tmpLibrary->GetTaskPlan()->GetTeam();

	for(auto agent : team) {
		auto& constraints = _node.GetMotionConstraints(_a2.m_task->GetParent(),agent);
		for(auto& c : constraints) {
			tmp->m_motionConstraintMap[agent].insert(std::make_pair(c.m_timestep,
																					std::make_pair(c.m_conflictCfg,c.m_duration)));
		}
	}

	Cfg startCfg = roadmap->GetVertex(startVID);
	startCfg.SetRobot(_a1.m_agent->GetRobot());
	Cfg endCfg = roadmap->GetVertex(goalVID);
	endCfg.SetRobot(_a1.m_agent->GetRobot());

	auto plan = tmp->MotionPlan(startCfg,endCfg,
															_a1.m_execEndTime,_a2.m_execStartTime,_a2.m_task->GetParent().get());

	tmp->m_motionConstraintMap.clear();

	if(plan.first > _a2.m_execStartTime) {
		return false;
	}

	_a2.m_setupPath = plan.second.first;
	_a2.m_setupWaitTimeSteps = plan.second.second;

	auto& taskPlan = _node.GetSolutionRef().m_taskPlans[_a2.m_task->GetParent()];
	for(auto& assign : taskPlan) {
		if(assign.m_agent != _a2.m_agent or
			 assign.m_execStartTime != _a2.m_execStartTime or
			 assign.m_execEndTime != _a2.m_execEndTime) {
			continue;
		}
		assign.m_setupPath = plan.second.first;
		assign.m_setupWaitTimeSteps = plan.second.second;
	}

	return true;
}
	
void
AllocationValidation::
PatchPaths(GeneralCBSNode& _node) {
	//Check to make sure that setup paths all line up since they can be changed to match a constraint
	//that no longer exists.
	auto& solution = _node.GetSolutionRef();
	auto& agentAssigns = solution.m_agentAssignments;

	auto tmp = static_cast<TMPLowLevelSearch*>(m_lowLevel);
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph("MTGraph").get());

	for(auto& kv : agentAssigns) {
		auto agent = kv.first;
		for(size_t i = 0; i < kv.second.size(); i++) {
			auto& a = kv.second[i];
			if(i == 0) {
				if(a.m_setupStartTime > 0) {
					auto g = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(a.m_agent));
					auto& constraints = _node.GetMotionConstraints(a.m_task->GetParent(),agent);
					for(auto& c : constraints) {
						tmp->m_motionConstraintMap[agent].insert(std::make_pair(c.m_timestep,
																					std::make_pair(c.m_conflictCfg,c.m_duration)));

						auto startCfg = agent->GetRobot()->GetSimulationModel()->GetState();
						startCfg.SetRobot(agent->GetRobot());
						auto endCfg = g->GetVertex(a.m_execPath.front());
						endCfg.SetRobot(agent->GetRobot());

						auto plan = tmp->MotionPlan(startCfg,endCfg,
										0,a.m_execStartTime,a.m_task->GetParent().get());

						if(plan.first > a.m_execStartTime)
							throw RunTimeException(WHERE, "James you broke something again.");

						
					}
				}
			}
		}
	}
}

