#include "PrecedenceLowLevelSearch.h"

/*--------------------------------------- Construction -------------------------------------*/
PrecedenceLowLevelSearch::
PrecedenceLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) : 
								TMPLowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel,_debug) { }

/*---------------------------------------- Interface ---------------------------------------*/

bool 
PrecedenceLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, SemanticTask* _task) {
	auto iter = _node.GetSolutionRef().m_subtaskFlow->GetFlowNodeIter(_task);
	auto vid = iter->descriptor();
	
	ClearFlowSubtree(_node,vid);

	UpdateCompletionTimes(_node);

	if(!PlanSubtree(_node,vid))
		return false;
	return ExtractPlan(_node);
}

/*------------------------------------- Helper Functions -----------------------------------*/

void 
PrecedenceLowLevelSearch::
ClearFlowSubtree(GeneralCBSNode& _node, size_t _vid) {
	auto flow = _node.GetSolutionRef().m_subtaskFlow;
	auto iter = flow->GetFlowNodeIter(_vid);
	auto& task = iter->property().m_task;

	_node.GetSolutionRef().m_taskPlans[task] = {};

	for(auto eit = iter->begin(); eit != iter->end(); eit++) {
		ClearFlowSubtree(_node, eit->target());
	}
}

void
PrecedenceLowLevelSearch::
UpdateCompletionTimes(GeneralCBSNode& _node) {
	auto& taskPlans = _node.GetSolutionRef().m_taskPlans;
	auto flow = _node.GetSolutionRef().m_subtaskFlow;

	for(auto vit = flow->begin(); vit != flow->end(); vit++) {
		if(!vit->property().m_task->GetMotionTask())
			continue;
		
		auto& tp = taskPlans[vit->property().m_task];
		if(tp.empty())
			m_taskCompletionTimes[vit->descriptor()] = MAX_DBL;
		else
			m_taskCompletionTimes[vit->descriptor()] = tp.back().m_execEndTime;
	}	
}

bool 
PrecedenceLowLevelSearch::
PlanSubtree(GeneralCBSNode& _node, size_t _start) {

	auto flow = _node.GetSolutionRef().m_subtaskFlow;

	std::priority_queue<std::pair<double,size_t>> pq;

	// Compute the available start time of the subsequent task.
	// Verify that all required preceeding tasks have been completed
	auto startTime = ComputeStartTime(_node, _start);
	if(startTime == MAX_DBL)
		return false;

	pq.push(std::make_pair(startTime,_start));

	// Keep track of which tasks have been computed.
	std::unordered_set<size_t> seen; 

	// Add the tasks which have an existing plan to the set.
	for(auto kv : m_taskCompletionTimes) {
		if(kv.second != MAX_DBL)
			seen.insert(kv.first);
	}

	while(!pq.empty() and seen.size() < flow->Size()) {
		auto current = pq.top();
		pq.pop();

		auto iter = flow->GetFlowNodeIter(current.second);

		seen.insert(current.second);

		// Update the task plan and check that it completeable.
		if(!UpdateIndividualTask(_node,iter->property().m_task,iter->descriptor(),current.first))
			continue;

		for(auto eit = iter->begin(); eit != iter->end(); eit++) {
			// Check that the task does not already have a better start time.
			if(seen.count(eit->target()))
				continue;

			// Compute the available start time of the subsequent task.
			// Verify that all required preceeding tasks have been completed.
			startTime = ComputeStartTime(_node, eit->target());
			if(startTime == MAX_DBL)
				continue;

			pq.push(std::make_pair(startTime, eit->target()));
		}
	}

	return seen.size() == flow->Size();
}

bool 
PrecedenceLowLevelSearch::
ExtractPlan(GeneralCBSNode& _node) {
	return true;
}

double
PrecedenceLowLevelSearch::
ComputeStartTime(GeneralCBSNode& _node, size_t _vid) {
	auto flow = _node.GetSolution().m_subtaskFlow;
	auto node = flow->GetFlowNode(_vid);
	auto func = node.m_initiationFunction;
	return EvaluateFunction(func);
}

double
PrecedenceLowLevelSearch::
EvaluateFunction(TBDFunction _func) {
	bool max = (_func.m_operator == TBDFunction::Operator::MAX);
	double value;
	max ? value = 0 : value = MAX_DBL;

	if(_func.m_subFunctions.empty()) {
		for(auto vid : _func.m_elems) {
			double eVal = m_taskCompletionTimes[vid];
			max ? value = std::max(value, eVal) : value = std::min(value, eVal);
		}
		return value;
	}
	
	for(auto sub : _func.m_subFunctions) {
		max ? value = std::max(value, EvaluateFunction(sub)) : value = std::min(value, EvaluateFunction(sub));
	}
	return value;
}

bool 
PrecedenceLowLevelSearch::
UpdateIndividualTask(GeneralCBSNode& _node, SemanticTask* _task, size_t _vid, double _precedence) {

	auto motion = _task->GetMotionTask();

	// Check that it's not the virtual root node.
	if(!motion) {
		m_taskCompletionTimes[_vid] = _precedence;
		return true;
	}
			
	// Temporarily adjust the time window to reflect the precedence constraints
	auto releaseWindow = motion->GetReleaseWindow();
	
	// Check if precedence constraint start time violates the time window.
	if(_precedence > releaseWindow.second)
		return false;

	// Save the window to restore after planning.
	auto oldWindow = releaseWindow;

	// Update the window to satisfy the precedence constraints
	releaseWindow.first = std::max(releaseWindow.first,_precedence);
	motion->SetReleaseWindow(releaseWindow);

	// Update the task solution.
	bool success = TMPLowLevelSearch::UpdateSolution(_node,_task);

	// Update local tracking of task completion time.
	if(success)
		m_taskCompletionTimes[_vid] = _node.GetSolutionRef().m_taskPlans[_task].back().m_execEndTime;
	
	// Restore the original time window.
	motion->SetReleaseWindow(oldWindow);
	
	return success;
}

/*----------------------------------------------------------------------------*/
