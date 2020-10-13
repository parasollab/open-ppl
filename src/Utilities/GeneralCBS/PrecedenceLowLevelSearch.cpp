#include "PrecedenceLowLevelSearch.h"

/*--------------------------------------- Construction -------------------------------------*/
PrecedenceLowLevelSearch::
PrecedenceLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) : 
								MetaTMPLowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel,_debug) { }

/*---------------------------------------- Interface ---------------------------------------*/

bool 
PrecedenceLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, SemanticTask* _task) {
	auto iter = _node.GetSolutionRef().m_subtaskFlow->GetFlowNodeIter(_task);
	auto vid = iter->descriptor();

	auto flow = _node.GetSolutionRef().m_subtaskFlow;
	vid = flow->GetSuperNode(vid);
	
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

	if(iter->descriptor() != _vid)
		throw RunTimeException(WHERE) << "Data corruption within SubtaskFlow. VID: " 
																	<< _vid
																	<< ", descriptor: " 
																	<< iter->descriptor()
																	<< std::endl;

	auto& task = iter->property().m_task;

	if(_node.GetSolutionRef().m_taskPlans[task].empty() and iter->property().m_subNodes.empty())
		return;	

	_node.GetSolutionRef().m_taskPlans[task] = {};

	for(auto eit = iter->begin(); eit != iter->end(); eit++) {
		ClearFlowSubtree(_node, eit->target());
	}
	for(auto s : iter->property().m_subNodes) {
		ClearFlowSubtree(_node, s);
	}
}

void
PrecedenceLowLevelSearch::
UpdateCompletionTimes(GeneralCBSNode& _node) {
	auto& taskPlans = _node.GetSolutionRef().m_taskPlans;
	auto flow = _node.GetSolutionRef().m_subtaskFlow;

	for(auto vit = flow->begin(); vit != flow->end(); vit++) {
		if(!vit->property().m_task->GetMotionTask()) {
			m_taskCompletionTimes[vit->descriptor()] = 0;
			continue;
		}
			
		auto& tp = taskPlans[vit->property().m_task];
		if(tp.empty())
			m_taskCompletionTimes[vit->descriptor()] = MAX_DBL;
		else
			m_taskCompletionTimes[vit->descriptor()] = tp.back().m_execEndTime;
	}	
	for(auto vit = flow->begin(); vit != flow->end(); vit++) {
		auto super = flow->GetSuperNode(vit->descriptor());
		if(super == vit->descriptor())
			continue;
		m_taskCompletionTimes[super] = std::max(m_taskCompletionTimes[super],m_taskCompletionTimes[vit->descriptor()]);
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

		// Check if it is a super node (holding synchronous tasks).
		if(!iter->property().m_subNodes.empty()) {
			if(!UpdateTaskGroup(_node, iter->property().m_subNodes, iter->descriptor(), current.first, seen, pq))
				continue;
		}

		// Update the task plan and check that it completeable.
		else if(!UpdateIndividualTask(_node,iter->property().m_task,iter->descriptor(),current.first))
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

	auto top = _node.GetSolutionRef().m_decomposition->GetMainTask();

	auto tasks = ExtractSubtreePlan(_node,top);
	
	_node.GetSolutionRef().m_solutionTasks = tasks.second;

	return true;
}

std::pair<double,std::unordered_set<SemanticTask*>>
PrecedenceLowLevelSearch::
ExtractSubtreePlan(GeneralCBSNode& _node, SemanticTask* _task) {
	if(_task->GetMotionTask()) {
		std::unordered_set<SemanticTask*> tasks;
		tasks.insert(_task);
		return std::make_pair(_node.GetSolutionRef().m_taskPlans[_task].back().m_execEndTime,tasks);
	}

	std::vector<std::pair<double,std::unordered_set<SemanticTask*>>> subtrees;
	double max = 0;
	double min = MAX_INT;
	size_t minIndex = MAX_INT;
	for(auto subtask : _task->GetSubtasks()) {
		auto subtree = ExtractSubtreePlan(_node,subtask);
		subtrees.push_back(subtree);
		max = std::max(max,subtree.first);
		if(min > subtree.first) {
			min = subtree.first;
			minIndex = subtrees.size()-1;
		}
	}
	
	if(_task->GetSubtaskRelation() == SemanticTask::AND) {
		std::unordered_set<SemanticTask*> tasks;
		for(auto subtree : subtrees) {
			for(auto task : subtree.second) {
				tasks.insert(task);
			}
		}
		return std::make_pair(max,tasks);
	}

	return subtrees[minIndex];
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
UpdateTaskGroup(GeneralCBSNode& _node, std::vector<size_t> _taskGroup, size_t _parentVID, double _precedence, 
									std::unordered_set<size_t>& _seen, std::priority_queue<std::pair<double,size_t>>& _pq) {

	auto& flow = _node.GetSolutionRef().m_subtaskFlow;

	//TODO::seperate into synchronization groups
	//call function on individual sync groups
	
	std::unordered_set<size_t> success;

	double release = _precedence;
	for(auto vid : _taskGroup) {
		auto iter = flow->GetFlowNodeIter(vid);
		release = std::max(release,iter->property().m_task->GetMotionTask()->GetReleaseWindow().first);
	}

	for(auto vid : _taskGroup) {
		auto iter = flow->GetFlowNodeIter(vid);
		if(iter->property().m_subNodes.empty() and
				UpdateIndividualTask(_node,iter->property().m_task,iter->descriptor(),release))//_precedence))
			success.insert(iter->descriptor());
		else 
			throw RunTimeException(WHERE) << "Recursive super nodes are not yet handled."
																		<< std::endl;
	}

	//Not all tasks within the synchronized group can be completed, therefore none can.
	if(success.size() < _taskGroup.size()){
		for(auto vid : success) {
			auto iter = flow->GetFlowNodeIter(vid);
			_node.GetSolutionRef().m_taskPlans[iter->property().m_task] = {};
		}
		return false;
	}

	// TODO::Assume synchronization means start at same time. Can also be start X timesteps later.
	bool sync = CheckSynchronization(_node,success);

	auto& taskPlans = _node.GetSolutionRef().m_taskPlans;

	if(sync) {
		double completion = 0;
		double initiation = MAX_DBL;
		for(auto vid : success) {
			auto iter = flow->GetFlowNodeIter(vid);
			completion = std::max(completion,taskPlans[iter->property().m_task].back().m_execEndTime);

			double& init = taskPlans[iter->property().m_task].front().m_execStartTime;
			if(initiation != MAX_DBL and initiation != init)
				throw RunTimeException(WHERE) << "Synchronization constraint violation was not caught." 
																			<< std::endl;
			initiation = std::min(initiation,init);
		}
		m_taskCompletionTimes[_parentVID] = completion;
		m_taskInitiationTimes[_parentVID] = initiation;

		for(auto vid : success) {
			_seen.insert(vid);

			auto iter = flow->GetFlowNodeIter(vid);

			for(auto eit = iter->begin(); eit != iter->end(); eit++) {
				
				// Check that the task does not already have a better start time.
				if(_seen.count(eit->target()) or success.count(eit->target()))
					continue;

				// Compute the available start time of the subsequent task.
				// Verify that all required preceeding tasks have been completed.
				double startTime = ComputeStartTime(_node, eit->target());
				if(startTime == MAX_DBL)
					continue;

				_pq.push(std::make_pair(startTime, eit->target()));
			}
		}
		return true;
	}

	//Recursively call function with updated start time
	//precedence = max of all task start times
	for(auto vid : _taskGroup) {
		auto iter = flow->GetFlowNodeIter(vid);
		_precedence = std::max(_precedence,taskPlans[iter->property().m_task].front().m_execStartTime);
		taskPlans[iter->property().m_task] = {};
	}

	return UpdateTaskGroup(_node,_taskGroup, _parentVID, _precedence, _seen, _pq);
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
	bool success = MetaTMPLowLevelSearch::UpdateSolution(_node,_task);

	// Update local tracking of task completion time.
	if(success) {
		m_taskCompletionTimes[_vid] = _node.GetSolutionRef().m_taskPlans[_task].back().m_execEndTime;
		m_taskInitiationTimes[_vid] = _node.GetSolutionRef().m_taskPlans[_task].front().m_execStartTime;
	}
	
	// Restore the original time window.
	motion->SetReleaseWindow(oldWindow);
	
	return success;
}

bool 
PrecedenceLowLevelSearch::
CheckSynchronization(GeneralCBSNode& _node, std::unordered_set<size_t> _tasks) {
	auto& taskPlans = _node.GetSolutionRef().m_taskPlans;
	auto flow = _node.GetSolutionRef().m_subtaskFlow;

	double initiation = MAX_DBL;
	for(auto vid : _tasks) {
		auto iter = flow->GetFlowNodeIter(vid);

		double& init = taskPlans[iter->property().m_task].front().m_execStartTime;
		if(initiation != MAX_DBL and initiation != init)
			return false;
		initiation = init;
	}
	
	return true;
}
/*----------------------------------------------------------------------------*/
