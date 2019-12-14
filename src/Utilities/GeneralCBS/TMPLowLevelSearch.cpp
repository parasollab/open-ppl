#include "TMPLowLevelSearch.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"
#include "ConfigurationSpace/Cfg.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TMPLibrary.h"

TMPLowLevelSearch::
TMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) : 
									LowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel,_debug) {}

/*----------------------------------- Interface -----------------------------------*/
bool
TMPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto query = sg->AddTaskToGraph(m_tmpLibrary->GetTaskPlan()->GetWholeTask(_task.get()));

	if(m_debug) {
		std::cout << "Updating Plan for: " << _task << std::endl;
		_node.Debug();
	std::cout << "Starting query: " << query << std::endl;
	}

	Initialize(_node, _task, query);

	if(m_debug) {
		for(auto inter : m_intervalMap) {
			auto vid = inter.first;
			auto cfg = sg->GetGraph()->GetVertex(vid);
			std::cout << "Intervals for " << vid << " : " << cfg.PrettyPrint() << std::endl;
			for(auto ai : inter.second) {
				std::cout << "\t" << ai.first->GetRobot()->GetLabel() << std::endl;
				for(auto i : ai.second) {
					std::cout << "\t\t\t" << i.first << "--->" << i.second << std::endl;
				}
			}
		}
	}


	auto plan = Search(_task,query);

	Uninitialize();

	if(plan.empty())
		return false;

	//TODO::change solution object within node
	_node.UpdateTaskPlan(_task,plan);

	if(m_debug) {
		std::cout << "Updated Plan for: " << _task << std::endl;
		_node.Debug();
	}

	return true;
}


/*-------------------------------- Helper Functions ------------------------------*/

void
TMPLowLevelSearch::
Initialize(GeneralCBSNode& _node, std::shared_ptr<SemanticTask> _task, std::pair<size_t,size_t> _query) {

	m_currentTask = _task.get();

	auto team = m_tmpLibrary->GetTaskPlan()->GetTeam();

	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	for(auto vit = sg->GetGraph()->begin(); vit != sg->GetGraph()->end(); vit++) {

		if(vit->descriptor() > sg->NonTaskNodes() and 
						(vit->descriptor() < _query.first or vit->descriptor() > _query.second+2))
			continue;

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
	for(auto agent : team) {
		auto& constraints = _node.GetMotionConstraints(_task,agent);
		for(auto& c : constraints) {
			m_motionConstraintMap[agent].insert(std::make_pair(c.m_timestep,
																					std::make_pair(c.m_conflictCfg,c.m_duration)));
		}
	}
}

std::vector<std::pair<double,double>>
TMPLowLevelSearch::
ComputeIntervals(GeneralCBSNode& _node, size_t _vid, std::shared_ptr<SemanticTask> _task, Agent* _agent) {
	//need to map interval to departure time/location pairing while finding avail intervals
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	//auto dummy = m_tmpLibrary->GetTaskPlan()->GetCapabilityAgent(_agent->GetCapability());

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
		/*
		startCfg.SetRobot(dummy->GetRobot());

		auto targetVID = g->GetVID(startCfg);
		if(targetVID != INVALID_VID) {

			RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EI eit;
			RoadmapGraph<Cfg,DefaultWeight<Cfg>>::VI vit;

			if(!g->find_edge(RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EID(_vid,targetVID),vit,eit))
				g->find_edge(RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EID(targetVID,_vid),vit,eit);

			departTime = startTime - eit->property().GetWeight();
		}*/
		auto vertexCfg = g->GetVertex(_vid);
		vertexCfg.SetRobot(_agent->GetRobot());
		//else {
			auto plan = MotionPlan(vertexCfg,startCfg,0,0);
			departTime = std::max(0.0,(startTime - plan.first));
		//}

		auto endCfg = constraint.m_endLocation;
		auto endTime = constraint.m_endTime;
		/*
		endCfg.SetRobot(dummy->GetRobot());

		auto sourceVID = g->GetVID(endCfg);
		if(sourceVID != INVALID_VID) {
			RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EI eit;
			RoadmapGraph<Cfg,DefaultWeight<Cfg>>::VI vit;

			if(!g->find_edge(RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EID(sourceVID,_vid),vit,eit))
				g->find_edge(RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EID(_vid,sourceVID),vit,eit);
			
			//RoadmapGraph<Cfg,DefaultWeight<Cfg>>::EP& edge = g->GetEdge(sourceVID,_vid);
			returnTime = endTime + eit->property().GetWeight();
		}
		else {*/
			plan = MotionPlan(endCfg,vertexCfg);//May need to perform a backwards search for this one
			returnTime = endTime + plan.first;
		//}

		AvailElem elem(_vid, _agent, init);

		auto iter = intervals.begin();
		//for(auto iter = intervals.begin(); iter != intervals.end(); iter++) {
		while(iter != intervals.end()) {
			/*if(*iter == elem.m_availInt) {
				m_availSourceMap[elem] = constraint;
				iter++;
				continue;
			}*/

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
				m_availEndMap[frontElem] = constraint;

				elem.m_availInt = *iter;
				m_availSourceMap[elem] = constraint;
				iter++;
				continue;
			}
			else if(iter->second <= returnTime and iter->first >= departTime) { //interval is enclosed within constraint
				auto temp = iter;
				iter++;
				intervals.erase(temp);
				continue;
			}
			else if(iter->second > departTime and iter->first < departTime) { //overlap is at the back of interval
				iter->second = departTime;

				//update which constraint is the source which interval
				elem.m_availInt = *iter;
				//m_availSourceMap[elem] = constraint;	
				m_availEndMap[elem] = constraint;
				iter++;
				continue;
			}
			else if(iter->first < returnTime and iter->second > returnTime) { //overlap is at the front of interval
				iter->first = returnTime;

				//update which constraint is the source which interval
				elem.m_availInt = *iter;
				m_availSourceMap[elem] = constraint;	
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
	m_usedAgents.clear();
	m_motionConstraintMap.clear();
	m_currentTask = nullptr;
}

std::vector<Assignment>
TMPLowLevelSearch::
Search(std::shared_ptr<SemanticTask> _task, std::pair<size_t,size_t> _query) {

	m_usedAgents.clear();
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
			//m_usedAgents[elem].insert(agentAvailInts.first);
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
			if(eit->target() > sg->NonTaskNodes() and 
						(eit->target() < _query.first or eit->target() > _query.second+2))
			continue;
			auto elems = ValidNeighbors(current.second, eit->target(), current.first, eit->property().GetWeight());
			for(auto elem : elems) {
				pq.push(elem);
				m_usedAgents[elem.second] = m_usedAgents[current.second];
				m_usedAgents[elem.second].insert(elem.second.m_agent);
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
		if(m_usedAgents[_elem].count(agent) and agent != _elem.m_agent)
			continue;
		for(auto avail : kv.second) {

			if(arrivalTime > avail.second)// or // too late
				//arrivalTime < avail.first)     // too early
				continue;

			if(_elem.m_availInt.second + _edgeCost < avail.first) // too early
				continue;

			AvailElem target(_vid,agent,avail);
			std::pair<double,std::pair<std::vector<size_t>,size_t>> transition;

			if(agent->GetRobot() == m_tmpLibrary->GetTaskPlan()->GetCoordinator()->GetRobot()) {
				//if(!CheckDeliveringAgentFutureConstraints(transition.first,_elem))  
				//	continue;
				transition.first = arrivalTime;
			}
			// interaction edge
			else if(agent->GetRobot()->GetCapability() != _elem.m_agent->GetRobot()->GetCapability()) { 
				transition = ComputeSetup(target,arrivalTime,_elem);
				if(transition.second.first.empty())
					continue;

				if(transition.first > _elem.m_availInt.second or // delivering agent has to leave
						transition.first > avail.second or // receiving agent has other obligations
						!CheckDeliveringAgentFutureConstraints(transition.first,_elem))  
					continue;

			}
			else if(agent == _elem.m_agent){ // motion subtask edge
				transition = ComputeExec(_elem, _vid, _currentCost);
				if(transition.second.first.empty())
					continue;

				if(transition.first > avail.second) // transition takes too long and violates a constraint
					continue;
			}
			else {
				continue;
			}

			//auto iter = m_seen.find(target);

			//TODO::Add custom ordering

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

std::pair<double,std::pair<std::vector<size_t>,size_t>>
TMPLowLevelSearch::
ComputeSetup(AvailElem _elem, double _minTime, AvailElem _parent) {	

	auto constraint = m_availSourceMap[_elem];
	auto startCfg = constraint.m_endLocation;
	auto startTime = constraint.m_endTime;
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	auto startVID = g->GetVID(startCfg);
	auto goalVID = _elem.m_vid;
	
	if(m_debug) {
		std::cout << "Computing setup path from " 
							<< startVID << startCfg.PrettyPrint() 
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
	Cfg goalCfg = g->GetVertex(goalVID);
	goalCfg.SetRobot(startCfg.GetRobot());
	auto plan = MotionPlan(startCfg,goalCfg,startTime, _minTime);

	auto path = plan.second;

	m_setupPathMap[_elem] = path;
	m_setupStartTimes[_elem] = startTime;


	if(plan.first > _minTime and _parent.m_agent != m_tmpLibrary->GetTaskPlan()->GetCoordinator()) {

		bool valid = CheckExec(_parent,plan.first,_elem);

		if(!valid) {
			path.first = {};
			path.second = 0;
			return std::make_pair(0,path);
		}
	}



	return std::make_pair(std::max(plan.first,_minTime),path);
}

std::pair<double,std::pair<std::vector<size_t>,size_t>>
TMPLowLevelSearch::
ComputeExec(AvailElem _elem, size_t _endVID, double _startTime) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();
 
	auto startVID = _elem.m_vid;

	if(m_debug) {
		std::cout << "Computing exec path from " 
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

	Cfg startCfg = g->GetVertex(startVID);
	startCfg.SetRobot(_elem.m_agent->GetRobot());
	Cfg goalCfg = g->GetVertex(_endVID);
	goalCfg.SetRobot(_elem.m_agent->GetRobot());
	
	auto plan  = MotionPlan(startCfg,goalCfg,_startTime);

	auto path = plan.second; 

	if(_elem.m_availInt.second != MAX_DBL) {
		auto constraint = m_availEndMap[_elem];
		auto endCfg = constraint.m_endLocation;
		if(endCfg.GetRobot() != goalCfg.GetRobot())
			throw RunTimeException(WHERE,"Mismatched robot types in constraint in exec path planning.");
		auto nextJobPlan = MotionPlan(goalCfg,endCfg,plan.first,constraint.m_startTime);
		if(nextJobPlan.first > constraint.m_startTime) {
			path.first = {};
			path.second = 0;
			return std::make_pair(0,path);
		}
	}	

	m_execPathMap[_elem] = path;
	return std::make_pair(plan.first,path);
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

			if(!planDetails.empty()) {
				auto& previous = planDetails.back();
				if(previous.m_execEndTime != m_distance[_plan[i-1]]) {
					auto rePath = m_reExecPathMap[elem][_plan[i-1]];
					previous.m_execPath = rePath.first;
					previous.m_finalWaitTimeSteps = rePath.second;
				}
			}

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

	auto setup = m_setupPathMap[_start];
	auto exec = m_execPathMap[_end];

	Assignment assign(_start.m_agent, semanticTask, setup.first, exec.first, 
										m_setupStartTimes[_start], m_distance[_start], _endTime, setup.second, exec.second);
	return assign;
}

std::pair<double,std::pair<std::vector<size_t>,size_t>>
TMPLowLevelSearch::
MotionPlan(Cfg _start, Cfg _goal, double _startTime, double _minEndTime, SemanticTask* _currentTask) {

	//moved this stuff into low level search
	/*m_currentRobot = _start.GetRobot();
	auto agent = m_currentRobot->GetAgent();
	m_currentMotionConstraints = &(m_motionConstraintMap[agent]);
	*/
	auto plan = this->LowLevelSearch::MotionPlan(_start,_goal,_startTime,_minEndTime,_currentTask);
/*
	m_currentRobot = nullptr;
	m_currentMotionConstraints = nullptr;
*/
	return plan;
}

bool 
TMPLowLevelSearch::
CheckDeliveringAgentFutureConstraints(double _interactionTime, AvailElem _elem) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();

	if(_elem.m_availInt.second != MAX_DBL) {
		auto startVID = _elem.m_vid;
		Cfg startCfg = g->GetVertex(startVID);
		startCfg.SetRobot(_elem.m_agent->GetRobot());

		auto constraint = m_availEndMap[_elem];

		if(!constraint.m_agent and !constraint.m_task)
			return true;

		auto endCfg = constraint.m_endLocation;
		if(endCfg.GetRobot() != startCfg.GetRobot())
			throw RunTimeException(WHERE,"Mismatched robot types in constraint in exec path planning.");
		auto nextJobPlan = MotionPlan(startCfg,endCfg,_interactionTime,constraint.m_startTime);
		if(nextJobPlan.first > constraint.m_startTime) {
			return false;
		}
	}	
	return true;
}

bool
TMPLowLevelSearch::
CheckExec(AvailElem _elem, double _endTime, AvailElem _post) {
	auto parent = m_parentMap[_elem];
	auto startTime = m_distance[parent];
	
	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	auto g = sg->GetGraph();
 
	auto startVID = parent.m_vid;

	if(m_debug) {
		std::cout << "Re-computing exec path from " 
							<< startVID 
							<< " to " 
							<< _elem.m_vid
							<< " for " 
							<< _elem.m_agent->GetRobot()->GetLabel() 
							<< std::endl;
	}	
	//Use motion planning low level approach here
	//Take in startVID, startTime, endVID, minEndTime

	//make sure double in return pair is the arrival time including the initial start time cost

	Cfg startCfg = g->GetVertex(startVID);
	startCfg.SetRobot(_elem.m_agent->GetRobot());
	Cfg goalCfg = g->GetVertex(_elem.m_vid);
	goalCfg.SetRobot(_elem.m_agent->GetRobot());
	
	auto plan  = MotionPlan(startCfg,goalCfg,startTime,_endTime);

	auto path = plan.second; 

	if(_elem.m_availInt.second != MAX_DBL) {
		auto constraint = m_availEndMap[_elem];
		auto endCfg = constraint.m_endLocation;
		if(endCfg.GetRobot() != goalCfg.GetRobot())
			throw RunTimeException(WHERE,"Mismatched robot types in constraint in exec path planning.");
		auto nextJobPlan = MotionPlan(goalCfg,endCfg,plan.first,constraint.m_startTime);
		if(nextJobPlan.first > constraint.m_startTime) {
			return false;
		}
	}	

	m_reExecPathMap[_post][_elem] = path;
	return true;
}
