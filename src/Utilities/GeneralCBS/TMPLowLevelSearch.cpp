#include "TMPLowLevelSearch.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"
#include "ConfigurationSpace/Cfg.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TMPLibrary.h"

TMPLowLevelSearch::
TMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel) : 
									m_tmpLibrary(_tmpLibrary), m_sgLabel(_sgLabel) {}

/*----------------------------------- Interface -----------------------------------*/
bool
TMPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto query = sg->AddTaskToGraph(m_tmpLibrary->GetTaskPlan()->GetWholeTask(_task.get()));

	std::cout << "Starting query: " << query << std::endl;

	Initialize(_node, _task);

	auto plan = Search(_task,query);

	Uninitialize();

	if(plan.empty())
		return false;

	//TODO::change solution object within node
	_node.UpdateTaskPlan(_task,plan);

	return true;
}


/*-------------------------------- Helper Functions ------------------------------*/

void
TMPLowLevelSearch::
Initialize(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {
	auto team = m_tmpLibrary->GetTaskPlan()->GetTeam();
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	for(auto vit = sg->GetGraph()->begin(); vit != sg->GetGraph()->end(); vit++) {
		auto& cfg = vit->property();
		size_t vid = vit->descriptor();
		if(cfg.GetRobot() == m_tmpLibrary->GetTaskPlan()->GetCoordinator()->GetRobot()) {
			m_intervalMap[vid][cfg.GetRobot()->GetAgent()] = {std::make_pair(0,MAX_DBL)};
			continue;
		}
		for(auto agent : team) {
			if(cfg.GetRobot()->GetCapability() == agent->GetRobot()->GetCapability()) {
				m_intervalMap[vid][agent] = ComputeIntervals(_node, vid, _task, agent);
			}
		}
	}
}

std::vector<std::pair<double,double>>
TMPLowLevelSearch::
ComputeIntervals(GeneralCBSNode& _node, size_t _vid, std::shared_ptr<SemanticTask> _task, Agent* _agent) {
	//need to map interval to departure time/location pairing while finding avail intervals
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	auto constraints = _node.GetAllocationConstraints(_task,_agent);

	AvailInterval init(0,MAX_DBL);
	AvailElem elem(_vid, _agent, init);
	AllocationConstraint blank;

	m_availSourceMap[elem] = blank;

	std::list<AvailInterval> intervals;
	intervals.push_back(init);

	double departTime;
	double returnTime;

	for(auto constraint : constraints) {
		auto startCfg = constraint.m_startLocation;
		auto startTime = constraint.m_startTime;

		//auto targetVID = g->GetVID(startCfg);
		//if(targetVID != INVALID_VID) {
		//	RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EP& edge = g->GetEdge(_vid,targetVID);
		//	departTime = startTime - edge.GetWeight();
		//}
		//else {
			auto plan = MotionPlan(g->GetVertex(_vid),startCfg);
			departTime = startTime - plan.first;
		//}

		auto endCfg = constraint.m_endLocation;
		auto endTime = constraint.m_endTime;
		
		//auto sourceVID = g->GetVID(endCfg);
		//if(sourceVID != INVALID_VID) {
		//	RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EP& edge = g->GetEdge(sourceVID,_vid);
		//	returnTime = endTime + edge.GetWeight();
		//}
		//else {
			plan = MotionPlan(endCfg,g->GetVertex(_vid));
			returnTime = endTime + plan.first;
		//}

		AvailElem elem(_vid, _agent, init);

		auto iter = intervals.begin();
		//for(auto iter = intervals.begin(); iter != intervals.end(); iter++) {
		while(iter != intervals.end()) {
			if(iter->second < departTime) { // depart time occurs after the interval ends
				iter++;
				continue;
			}
			if(iter->first > returnTime) { //return time occurs before the interval starts
				iter++;
				continue;			
			}

			elem.m_availInt = *iter;

			//alloc constraint interferes with this interval
			
			if(iter->second > returnTime and iter->first < departTime) { //constraint is enclosed within interval
				AvailInterval front = std::make_pair(iter->first, departTime);
				intervals.insert(iter,front);
				iter->first = returnTime;

				//update which constraint is the source which interval

				AvailElem frontElem = elem;
				frontElem.m_availInt = front;

				m_availSourceMap[frontElem] = m_availSourceMap[elem];

				elem.m_availInt = *iter;
				m_availSourceMap[elem] = constraint;
				std::cout << "Here1" << std::endl;
				iter++;
				continue;
			}
			else if(iter->second <= returnTime and iter->first >= departTime) { //interval is enclosed within constraint
				auto temp = iter;
				iter++;
				intervals.erase(temp);
				std::cout << "Here2" << std::endl;
				continue;
			}
			else if(iter->second > departTime and iter->first < departTime) { //overlap is at the back of interval
				iter->second = departTime;

				//update which constraint is the source which interval
				elem.m_availInt = *iter;
				m_availSourceMap[elem] = constraint;	
				std::cout << "Here3" << std::endl;
				iter++;
				continue;
			}
			else if(iter->first < returnTime and iter->second > returnTime) { //overlap is at the front of interval
				iter->first = returnTime;

				//update which constraint is the source which interval
				elem.m_availInt = *iter;
				m_availSourceMap[elem] = constraint;	
				std::cout << "Here4" << std::endl;
				iter++;
				continue;
			}
			iter++;
		}
	}

	std::vector<AvailInterval> availInts;
	for(auto avail : intervals) {
					availInts.push_back(avail);
	}

	return availInts;
}

void
TMPLowLevelSearch::
Uninitialize() {
	m_parentMap.clear();
	m_intervalMap.clear();
	m_availSourceMap.clear();
	m_execPathMap.clear();
	m_setupPathMap.clear();
	m_setupStartTimes.clear();
	m_seen.clear();
	m_visited.clear();
	m_distance.clear();
}

std::vector<Assignment>
TMPLowLevelSearch::
Search(std::shared_ptr<SemanticTask> _task, std::pair<size_t,size_t> _query) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	size_t start = _query.first;
	size_t goal = _query.second;
	//TODO::check for arbitrary start == goal path

	std::priority_queue<std::pair<double,AvailElem>,
											std::vector<std::pair<double,AvailElem>>,
											std::greater<std::pair<double,AvailElem>>> pq;

	for(auto agentAvailInts : m_intervalMap[start]) {
		for(auto avail : agentAvailInts.second) {
			AvailElem elem(start, agentAvailInts.first, avail);
			pq.push(std::make_pair(0,elem));
		}
	}

	std::pair<double,AvailElem> current;

	std::set<size_t> debug;

	do {

		current = pq.top();
		pq.pop();

		if(m_visited.count(current.second))
			continue;

		m_visited.insert(current.second);

		auto vit = sg->GetGraph()->find_vertex(current.second.m_vid);
		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			auto elems = ValidNeighbors(current.second, eit->target(), current.first, eit->property().GetWeight());
			for(auto elem : elems) {
				pq.push(elem);
				debug.insert(elem.second.m_vid);
			} 
		}


	} while(!pq.empty() and current.second.m_vid != goal);

	if(current.second.m_vid != goal)
		return {};

	vector<AvailElem> plan;

	auto elem = current.second;
	plan.push_back(elem);

	do {
		elem = m_parentMap[elem];
		plan.push_back(elem);
	} while(elem.m_vid != start);

	std::reverse(plan.begin(),plan.end());

	return PlanDetails(plan, _task);

}
	
std::vector<std::pair<double,TMPLowLevelSearch::AvailElem>> 
TMPLowLevelSearch::
ValidNeighbors(const AvailElem& _elem, size_t _vid, double _currentCost, double _edgeCost) {

	std::vector<std::pair<double,AvailElem>> validNeighbors;

	double arrivalTime = _currentCost + _edgeCost;

	for(auto kv : m_intervalMap[_vid]) {
		auto agent = kv.first;
		for(auto avail : kv.second) {
			if(arrivalTime > avail.second)// or // too late
				//arrivalTime < avail.first)     // too early
				continue;

			if(_elem.m_availInt.second + _edgeCost < avail.first) // too early
				continue;

			AvailElem target(_vid,agent,avail);
			std::pair<double,std::vector<size_t>> transition;

			if(agent->GetRobot() == m_tmpLibrary->GetTaskPlan()->GetCoordinator()->GetRobot()) {
				transition.first = arrivalTime;
			}
			// interaction edge
			else if(agent->GetRobot()->GetCapability() != _elem.m_agent->GetRobot()->GetCapability()) { 
				transition = ComputeSetup(target,arrivalTime);
				if(transition.second.empty())
					continue;

				if(transition.first > _elem.m_availInt.second or // delivering agent has to leave
						transition.first > avail.second) // receiving agent has other obligations 
					continue;

			}
			else if(agent == _elem.m_agent){ // motion subtask edge
				transition = ComputeExec(_elem, _vid, _currentCost);
				if(transition.second.empty())
					continue;

				if(transition.first > avail.second) // transition takes too long and violates a constraint
					continue;
			}
			else {
				continue;
			}

			//auto iter = m_seen.find(target);

			//if(iter == m_seen.end() or m_distance[target] > transition.first) {
			if(!m_seen.count(target) or m_distance[target] > transition.first) {
			//if(!m_distance[target] == 0 or m_distance[target] > transition.first) {
				validNeighbors.push_back(std::make_pair(transition.first,target));
				m_seen.insert(target);
				m_parentMap[target] = _elem;
				m_distance[target] = transition.first;
				if(agent == _elem.m_agent) {
					m_execPathMap[target] = transition.second;
				}
				else {
					m_setupPathMap[target] = transition.second;
				}
			}	
		}
	}

	return validNeighbors;
}

std::pair<double,std::vector<size_t>>
TMPLowLevelSearch::
ComputeSetup(AvailElem _elem, double _minTime) {	
	vector<size_t> path;

	auto constraint = m_availSourceMap[_elem];
	auto startCfg = constraint.m_endLocation;
	auto startTime = constraint.m_endTime;
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	auto startVID = g->GetVID(startCfg);
	auto goalVID = _elem.m_vid;
	
	if(m_debug) {
		std::cout << "Computing setup path from " 
							<< startVID 
							<< " to " 
							<< goalVID 
							<< " for " 
							<< _elem.m_agent->GetRobot()->GetLabel() 
							<< ".\nStarting at: "
							<< startTime 
							<< std::endl;
	}	
	//Use motion planning low level approach here
	//Take in startVID, startTime, endVID, minEndTime

	//make sure the cost accounts for the start time and the transition time
	
	//temp until motion stuff is ready
	auto plan = MotionPlan(startCfg,g->GetVertex(goalVID));

	path = plan.second;

	m_setupPathMap[_elem] = path;
	m_setupStartTimes[_elem] = startTime;
	return std::make_pair(std::max(plan.first+startTime,_minTime),path);
}

std::pair<double,std::vector<size_t>>
TMPLowLevelSearch::
ComputeExec(AvailElem _elem, size_t _endVID, double _startTime) {
	vector<size_t> path;

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();
 
	auto startVID = _elem.m_vid;

	if(m_debug) {
		std::cout << "Computing setup path from " 
							<< startVID 
							<< " to " 
							<< _endVID 
							<< " for " 
							<< _elem.m_agent->GetRobot()->GetLabel() 
							<< std::endl;
	}	
	//Use motion planning low level approach here
	//Take in startVID, startTime, endVID, minEndTime

	//make sure double in return pair is the arrival time including the initial start time cost

	auto plan  = MotionPlan(g->GetVertex(startVID),g->GetVertex(_endVID));

	path = plan.second; 

	m_execPathMap[_elem] = path;
	return std::make_pair(plan.first+_startTime,path);
}

std::vector<Assignment> 
TMPLowLevelSearch::
PlanDetails(std::vector<AvailElem> _plan, std::shared_ptr<SemanticTask> _task) {

	std::vector<Assignment> planDetails;

	Agent* agent = nullptr;
	AvailElem start;

	for(size_t i = 1; i < _plan.size()-1; i++) {
		AvailElem elem = _plan[i];

		if(elem.m_agent != agent) {
			agent = elem.m_agent;
			start = elem;
			continue;
		}

		double endTime = m_distance[elem];
		if(i < _plan.size() - 2) {
			endTime = m_distance[_plan[i+1]];
		}

		Assignment assign = CreateAssignment(start,elem,_task,endTime);
		//TODO::Add in sequential dependency notion to semantic tasks here when we start caring
		planDetails.push_back(assign);
	}

	return planDetails;
}

Assignment 
TMPLowLevelSearch::
CreateAssignment(AvailElem _start, AvailElem _end, std::shared_ptr<SemanticTask> _parentTask, double _endTime) {
	if(_start.m_agent != _end.m_agent)
		throw RunTimeException(WHERE, "Attempting to create assignment with mismatching agent avail-task nodes.");

	auto robot = _start.m_agent->GetRobot();

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	std::shared_ptr<MPTask> motionTask = std::shared_ptr<MPTask>(new MPTask(robot));

	//TODO::make sure that this is the right cfg for this robot
	auto startCfg = g->GetVertex(_start.m_vid);
	if(startCfg.GetRobot()->GetCapability() != robot->GetCapability())
		throw RunTimeException(WHERE, "Cfg and robot are of mismatching types.");

	startCfg.SetRobot(robot);
	auto startConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,startCfg));
 
	auto goalCfg = g->GetVertex(_end.m_vid);
	if(goalCfg.GetRobot()->GetCapability() != robot->GetCapability())
		throw RunTimeException(WHERE, "Cfg and robot are of mismatching types.");

	goalCfg.SetRobot(robot);
	auto goalConstraint = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(robot,goalCfg));

	motionTask->SetStartConstraint(std::move(startConstraint));
	motionTask->AddGoalConstraint(std::move(goalConstraint));

	std::shared_ptr<SemanticTask> semanticTask = std::shared_ptr<SemanticTask>(new SemanticTask(_parentTask,motionTask));

	Assignment assign(_start.m_agent, semanticTask, m_setupPathMap[_start], m_execPathMap[_end], 
										m_setupStartTimes[_start], m_distance[_start], _endTime);
	return assign;
}

std::pair<double,std::vector<size_t>>
TMPLowLevelSearch::
MotionPlan(Cfg _start, Cfg _goal) {
  if(_start.GetRobot()->GetCapability() != _goal.GetRobot()->GetCapability()){
    throw RunTimeException(WHERE, "start and goal are of mismatched robot types.");
  }
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

  //save current library task
  auto oldTask = m_tmpLibrary->GetMPLibrary()->GetTask();

  auto dummyAgent = m_tmpLibrary->GetTaskPlan()->GetCapabilityAgent(_start.GetRobot()->GetCapability());
  auto robot = dummyAgent->GetRobot();

  _start.SetRobot(robot);
  _goal.SetRobot(robot);

  std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robot));

  std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(robot, _start));
  std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(robot, _goal));

  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  m_tmpLibrary->GetMPLibrary()->SetTask(task.get());

  auto solution = new MPSolution(dummyAgent->GetRobot());

	auto roadmap = sg->GetCapabilityRoadmap(static_cast<HandoffAgent*>(robot->GetAgent()));

  solution->SetRoadmap(dummyAgent->GetRobot(),roadmap.get());

  m_tmpLibrary->GetMPLibrary()->Solve(m_tmpLibrary->GetMPLibrary()->GetMPProblem(), task.get(), solution, "EvaluateMapStrategy",
      LRand(), "LowerLevelGraphWeight");

  if(m_tmpLibrary->GetMPLibrary()->GetPath()->Cfgs().empty())
    return {};

  //restore library task
  m_tmpLibrary->GetMPLibrary()->SetTask(oldTask);
	
	return std::make_pair(solution->GetPath()->Length(),solution->GetPath(robot)->VIDs());
}
