#include "MPLowLevelSearch.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"

MPLowLevelSearch::
MPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel) : 
									LowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel) {}

bool
MPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {

	std::vector<Assignment>& assignments = _node.GetSolutionRef().m_taskPlans[_task];

	Assignment assign;

	for(auto& a : assignments) {
		if(!a.m_execPath.empty() and a.m_agent != nullptr) {
			continue;	
		}
		if(!a.m_agent)
			throw RunTimeException(WHERE,"Task plan to update has no cleared assignment plans.");
		assign = a;
		break;
	}

	ClearTaskPlan(assign,_node);

	return PlanAssignments(_node);
}

void
MPLowLevelSearch::
ClearTaskPlan(Assignment& _assign, GeneralCBSNode& _node) {

	if(_assign.m_execPath.empty())
		return;
	
	_assign.m_execPath = {};

	std::vector<Assignment>& assignments = _node.GetSolutionRef().m_taskPlans[_assign.m_task->GetParent()];
	bool before = true;
	for(auto& a : assignments) {
		if(a == _assign)
			before = false;
		if(before)
			continue;
		ClearAgentAssignments(a, _node);
	}
}

void
MPLowLevelSearch::
ClearAgentAssignments(Assignment& _assign, GeneralCBSNode& _node) {
	
	if(_assign.m_execPath.empty())
		return;
	
	_assign.m_execPath = {};

	std::vector<Assignment>& assignments = _node.GetSolutionRef().m_agentAssignments[_assign.m_agent];
	bool before = true;
	for(auto& a : assignments) {
		if(a == _assign) 
			before = false;
		if(before) 
			continue;
		ClearTaskPlan(a, _node);
	}
}

bool 
MPLowLevelSearch::
PlanAssignments(GeneralCBSNode& _node) {

	auto& taskPlans = _node.GetSolutionRef().m_taskPlans;
	auto& agentAssignments = _node.GetSolutionRef().m_agentAssignments;

	size_t resolved = 0;
	while(resolved < taskPlans.size()) {
		for(auto& kv : taskPlans) {
			for(size_t i = 0; kv.second.size(); i++) {
				auto& assign = kv.second[i];
				auto agent = assign.m_agent;
				bool ready = false;
				auto& assignments = agentAssignments[agent];
				size_t j;
				for(j=0; j < assignments.size(); j++) {
					if(assign != assignments[j])
						continue;
					if(!assignments[j-1].m_execPath.empty())
						ready = true;
					break;
				}
				if(!ready)
					continue;
				if(i == 0){
					Assignment temp;
					PlanAssignment(_node, assign, temp);
				}
				else {
					PlanAssignment(_node, assign, assignments[j-1], kv.second[i-1].m_execEndTime);
				}
			}
		}
	}

	return true;
}

bool
MPLowLevelSearch::
PlanAssignment(GeneralCBSNode& _node, Assignment& _assign, Assignment& _previous, 
							  double startTime, double endTime) {

	auto agent = _assign.m_agent;
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	Cfg setupCfg;
	if(!_previous.m_agent)
		setupCfg = agent->GetRobot()->GetSimulationModel()->GetState();
	else
		setupCfg = g->GetVertex(_previous.m_execPath.back());

	Cfg startCfg = m_tmpLibrary->GetTaskPlan()->GetWholeTask(
												_assign.m_task.get())->m_startPoints[agent->GetRobot()->GetCapability()][0];
	Cfg goalCfg = m_tmpLibrary->GetTaskPlan()->GetWholeTask(
												_assign.m_task.get())->m_goalPoints[agent->GetRobot()->GetCapability()][0];

	auto setup = this->MotionPlan(setupCfg,startCfg,startTime,0);

	if(setup.second.empty())
		return false;

	auto exec = this->MotionPlan(startCfg,goalCfg,setup.first,0);

	if(exec.second.empty())
		return false;

	_assign.m_setupPath = setup.second;
	_assign.m_execPath = exec.second;

	_assign.m_execStartTime = setup.first;
	_assign.m_execEndTime = exec.first;	

	return true;	
} 


