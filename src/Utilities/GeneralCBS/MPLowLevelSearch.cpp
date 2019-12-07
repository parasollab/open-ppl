#include "MPLowLevelSearch.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"

MPLowLevelSearch::
MPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) : 
									LowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel, _debug) {}

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

	if(m_debug) {
		std::cout << "Pre-Clearing task plan" << std::endl;
		auto solution = _node.GetSolution();
		auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
		for(auto plan : solution.m_taskPlans) {
			auto task = plan.first;
			auto assignments = plan.second;
			std::cout << std::endl << std::endl << "Task: " << task << std::endl;
			for(auto a : assignments) {
				if(!a.m_agent)
					continue;
				std::cout << "\tAgent: " << a.m_agent->GetRobot()->GetLabel() << std::endl;
				auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(a.m_agent)).get();
				std::cout << "\t\tSetup Path" << std::endl;
				for(auto vid : a.m_setupPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
				std::cout << "\t\tExec Path    (" << a.m_execStartTime << "->"  << a.m_execEndTime << ")" << std::endl;
				for(auto vid : a.m_execPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
			}
		}
	}
	ClearTaskPlan(assign,_node);
	if(m_debug) {
		std::cout << "Post-Clearing task plan" << std::endl;
		auto solution = _node.GetSolution();
		auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
		for(auto plan : solution.m_taskPlans) {
			auto task = plan.first;
			auto assignments = plan.second;
			std::cout << std::endl << std::endl << "Task: " << task << std::endl;
			for(auto a : assignments) {
				if(!a.m_agent)
					continue;
				std::cout << "\tAgent: " << a.m_agent->GetRobot()->GetLabel() << std::endl;
				auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(a.m_agent)).get();
				std::cout << "\t\tSetup Path" << std::endl;
				for(auto vid : a.m_setupPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
				std::cout << "\t\tExec Path    (" << a.m_execStartTime << "->"  << a.m_execEndTime << ")" << std::endl;
				for(auto vid : a.m_execPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
			}
		}
	}

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
		if(!a.m_agent)
			break;
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

	std::set<SemanticTask*> resolved;

	while(resolved.size() < taskPlans.size()) {
		for(auto& kv : taskPlans) {
			bool ready;
			for(size_t i = 0; i < kv.second.size(); i++) {
				auto& assign = kv.second[i];

				//Check if all allocated subtasks have been planned for.
				if(!assign.m_agent) {
					resolved.insert(kv.first.get());
					break;
				}

				//Check if this subtask has been planned already.
				if(!assign.m_execPath.empty()) {
					if(i == kv.second.size()-1)
						resolved.insert(kv.first.get());
					continue;
				}

				auto agent = assign.m_agent;
				auto& assignments = agentAssignments[agent];
				size_t j;

				//Check if the agent is able to be planned for yet.
				ready = false;
				if(assignments.empty())
					ready = true;

				for(j=0; j < assignments.size(); j++) {
					if(assign != assignments[j])
						continue;
					if(j == 0 and assign == assignments[j])
						ready = true;
					else if(!assignments[j-1].m_execPath.empty())
						ready = true;
					break;
				}

				if(!ready)
					break;
				if(i == 0 and j == 0){
					Assignment temp;
					if(!PlanAssignment(_node, assign, temp))
						return false;
				}
				else if(i == 0) {
					if(!PlanAssignment(_node, assign, assignments[j-1], 0))
						return false;
				}
				else if(j==0){
					Assignment temp;
					if(!PlanAssignment(_node, assign, temp, kv.second[i-1].m_execEndTime))
						return false;
				}
				else {
					if(!PlanAssignment(_node, assign, assignments[j-1], kv.second[i-1].m_execEndTime))
						return false;
				}

				if(!assignments.empty()) {
					//assignments[j] = assign;
					assignments[j].m_setupPath = assign.m_setupPath;
					assignments[j].m_execPath = assign.m_execPath;
					assignments[j].m_execStartTime = assign.m_execStartTime;
					assignments[j].m_execEndTime = assign.m_execEndTime;
					assignments[j].m_setupStartTime = assign.m_setupStartTime;
				}

				if(i == kv.second.size()-1)
					resolved.insert(kv.first.get());
			}
		}
		if(m_debug) {
			std::cout << resolved << " tasks resolved out of " << taskPlans.size() << std::endl;
		}
	}

	if(m_debug) {
		std::cout << "Post-Replanning" << std::endl;
		auto solution = _node.GetSolution();
		auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
		for(auto plan : solution.m_taskPlans) {
			auto task = plan.first;
			auto assignments = plan.second;
			std::cout << std::endl << std::endl << "Task: " << task << std::endl;
			for(auto a : assignments) {
				if(!a.m_agent)
					continue;
				std::cout << "\tAgent: " << a.m_agent->GetRobot()->GetLabel() << std::endl;
				auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(a.m_agent)).get();
				std::cout << "\t\tSetup Path" << std::endl;
				for(auto vid : a.m_setupPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
				std::cout << "\t\tExec Path    (" << a.m_execStartTime << "->"  << a.m_execEndTime << ")" << std::endl;
				for(auto vid : a.m_execPath) {
					std::cout << "\t\t\t" << roadmap->GetVertex(vid).PrettyPrint() << std::endl;
				}
			}
		}
	}

	double cost = 0;
	for(auto tp : _node.GetSolution().m_taskPlans) {
		double taskCost = 0;
		for(auto a : tp.second) {
 			if(!a.m_agent) {
				break;
			}
			taskCost = a.m_execEndTime;	
		}
		cost += taskCost;
	}

	_node.SetCost(cost);
	return true;
}

bool
MPLowLevelSearch::
PlanAssignment(GeneralCBSNode& _node, Assignment& _assign, Assignment& _previous, 
							  double _startTime, double _endTime) {

	auto& constraints = _node.GetMotionConstraints(_assign.m_task->GetParent(),_assign.m_agent);
	for(auto& c : constraints) {
		m_motionConstraintMap[_assign.m_agent].insert(std::make_pair(c.m_timestep,c.m_conflictCfg));
	}

	auto agent = _assign.m_agent;
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(agent));

	Cfg setupCfg;
	double setupStart;
	if(!_previous.m_agent) {
		setupCfg = agent->GetRobot()->GetSimulationModel()->GetState();
		setupStart = 0;
	}
	else {
		setupCfg = roadmap->GetVertex(_previous.m_execPath.back());
		setupCfg.SetRobot(agent->GetRobot());
		setupStart = _previous.m_execEndTime;
	}

	auto query = m_tmpLibrary->GetTaskPlan()->GetWholeTask(
									_assign.m_task->GetParent().get())->m_subtaskStartEndCfgs[_assign.m_task->GetMotionTask()];

	//Agent is does not have a configuration at task start.
	if(!query.first.GetRobot())
		return false;

	//Agent is does not have a configuration at task goal.
	if(!query.second.GetRobot())
		return false;

	//Agent is does not have a configuration at task start.
	if(query.first.GetRobot()->GetCapability() != agent->GetCapability())
		return false;

	query.first.SetRobot(agent->GetRobot());
	query.second.SetRobot(agent->GetRobot());

	auto setup = this->MotionPlan(setupCfg,query.first,setupStart,_startTime);

	if(setup.second.empty())
		return false;

	//TODO::Check if preceeding task needs to be patched to account for waiting time
	if(_startTime > 0 
			and setup.first > _startTime 
			and !PatchPaths(_node,_assign,setupCfg,query.first,setupStart))
		return false;

	auto exec = this->MotionPlan(query.first,query.second,setup.first,0);

	if(exec.second.empty())
		return false;

	_assign.m_setupPath = setup.second;
	_assign.m_execPath = exec.second;

	_assign.m_execStartTime = setup.first;
	_assign.m_execEndTime = exec.first;

	_assign.m_setupStartTime = _startTime;	

	return true;	
} 

bool
MPLowLevelSearch::
PatchPaths(GeneralCBSNode& _node, Assignment& _assign, Cfg _setupCfg, Cfg _startCfg, double _setupStart) {
	return true;
}
