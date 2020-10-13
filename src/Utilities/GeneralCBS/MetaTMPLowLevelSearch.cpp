#include "MetaTMPLowLevelSearch.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"

/*----------------------------------------- Construction ---------------------------------------*/

MetaTMPLowLevelSearch::
MetaTMPLowLevelSearch(TMPLibrary* _tmpLibrary, std::string _sgLabel, std::string _vcLabel, bool _debug) :
											TMPLowLevelSearch(_tmpLibrary,_sgLabel,_vcLabel,_debug) {}

MetaTMPLowLevelSearch::
~MetaTMPLowLevelSearch() {}

/*------------------------------------------ Interface ----------------------------------------*/

bool
MetaTMPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, SemanticTask* _task) {

	if(!_node.GetSolutionRef().m_metaTaskMap[_task])
		return TMPLowLevelSearch::UpdateSolution(_node,_task);

	Initialize(_node,_task,std::make_pair(0,0));

	bool success = Search(_node,_node.GetSolutionRef().m_metaTaskMap[_task]);

	Uninitialize();

	if(m_debug) {
		if(success)
			std::cout << "Successfully ";
		else 
			std::cout << "Unsuccessfully ";
		std::cout << "updated Plan for:  ";
		for(auto task : _node.GetSolutionRef().m_metaTaskMap[_task]->GetSubtasks()){
			std::cout << task->GetLabel() << ", ";
		}
		std::cout << std::endl;
		_node.Debug();
	}
	return success;
}

/*--------------------------------------- Helper Functions ------------------------------------*/

void
MetaTMPLowLevelSearch::
Initialize(GeneralCBSNode& _node, SemanticTask* _task, std::pair<size_t,size_t> _query) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	auto metaTask = _node.GetSolutionRef().m_metaTaskMap[_task];

	for(auto task : metaTask->GetSubtasks()) {
		auto query = sg->AddTaskToGraph(m_tmpLibrary->GetTaskPlan()->GetWholeTask(task));
		this->TMPLowLevelSearch::Initialize(_node,task,query);
	}

	if(m_debug) {
		std::cout << "Elems with constraints" << std::endl;
		//for(auto kv : m_availSourceMap) {
			//std::cout << "Task: " << kv.first->GetLabel() << std::endl;
			for(auto kv2 : m_availSourceMap) {
				std::cout << kv2.first.m_vid
									<< ", "
									<< kv2.first.m_agent->GetRobot()->GetLabel()
									<< ", "
									<< kv2.first.m_availInt
									<< std::endl;
			}
		//}
		//for(auto kv : m_intervalMaps) {
			//std::cout << "Task: " << kv.first->GetLabel() << std::endl;
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
		//}
		std::cout << "Done Printing debug stuff." << std::endl;
	}
}

void
MetaTMPLowLevelSearch::
Uninitialize() {
	this->TMPLowLevelSearch::Uninitialize();
	m_parentMap.clear();	
	m_distance.clear();
	m_seen.clear();
	m_visited.clear();
}

bool
MetaTMPLowLevelSearch::
Search(GeneralCBSNode& _node, SemanticTask* _metaTask) {

	m_usedAgents.clear();

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	std::priority_queue<std::pair<double,MetaElem>,
											std::vector<std::pair<double,MetaElem>>,
											MetaElemGreaterThan> pq;

	std::unordered_map<SemanticTask*,size_t> goal;
	std::unordered_map<SemanticTask*,size_t> start;

	std::unordered_map<SemanticTask*,std::vector<AvailElem>> startElems;
	for(auto task : _metaTask->GetSubtasks()) {
		auto query = sg->AddTaskToGraph(m_tmpLibrary->GetTaskPlan()->GetWholeTask(task));

		start[task] = query.first;
		goal[task] = query.second;

		for(auto agentAvailInts : m_intervalMap[query.first]) {
			for(auto avail : agentAvailInts.second) {
				AvailElem elem(query.first, agentAvailInts.first, avail);
				//auto coordinator = static_cast<Agent*>(m_tmpLibrary->GetTaskPlan()->GetCoordinator());
				//auto release = m_intervalMap[query.first][coordinator][0].first;
				startElems[task].push_back(elem);
			}
		}
	}

	std::vector<MetaElem> startMetaElems;
	for(auto task : _metaTask->GetSubtasks()) {
		std::vector<MetaElem> temp;

		if(startMetaElems.empty()) {
			for(auto start : startElems[task]) {
				MetaElem meta;
				meta[task] = start;
				startMetaElems.push_back(meta);
			}	
			continue;
		}

		for(auto start : startElems[task]) {

			for(auto meta : startMetaElems) {
				meta[task] = start;
				temp.push_back(meta);
			}
		}
		startMetaElems = temp;
	}

	for(auto meta : startMetaElems) {
		double min = MAX_DBL;
		double soc = 0;
		m_distance[meta] = {};
		for(auto kv : meta) {
			min = std::min(min,kv.second.m_availInt.first);
			soc += kv.second.m_availInt.first;
			m_distance[meta][kv.first] = kv.second.m_availInt.first;
		}
		//pq.push(std::make_pair(min,meta));
		pq.push(std::make_pair(soc,meta));
	}

	bool foundGoal = true;

	std::pair<double,MetaElem> current;

	while(!pq.empty()) {
		current = pq.top();
		pq.pop();

		if(m_visited.count(current.second)){
			if(m_debug) {
				for(auto m : m_visited) {
					if(MetaElemCompare()(m,current.second) == MetaElemCompare()(current.second,m)){
						auto o = MetaElemCompare()(m,current.second);
						auto t = MetaElemCompare()(current.second,m);
						std::cout << "HERE" << o << t << std::endl;
					}
				}
			}
			continue;
		}

		m_visited.insert(current.second);

		// Check if all tasks have reached their goal
		foundGoal = true;
		for(auto kv : current.second) {
			if(kv.second.m_vid != goal[kv.first]) {
				foundGoal = false;
				break;
			}
		}
		if(foundGoal)
			break;

		for(auto kv : current.second) {
			auto vit = sg->GetGraph()->find_vertex(kv.second.m_vid);
			for(auto eit = vit->begin(); eit != vit->end(); eit++) {
				if(eit->target() > sg->NonTaskNodes() and 
						(eit->target() < goal[kv.first] or eit->target() > goal[kv.first]+2))
					continue;

				auto metas = ValidMetaNeighbors(current.second,kv.first, 
															eit->target(),current.first,eit->property().GetWeight());
				for(auto meta : metas) {
					pq.push(meta);
				}
			}
		}

	}

	if(!foundGoal)
		return false;

	std::vector<MetaElem> plan;

	auto meta = current.second;
	plan.push_back(meta);	

	bool reachedStart = false;

	do {
		meta = m_parentMap[meta];
		plan.push_back(meta);
		reachedStart = true;
		for(auto kv : meta) {
			if(kv.second.m_vid != start[kv.first]) {
				reachedStart = false;
				break;
			}
		}
	} while(!reachedStart);

	std::reverse(plan.begin(),plan.end());

	UpdateSolution(_node,plan);

	return true;
}

std::vector<std::pair<double,MetaTMPLowLevelSearch::MetaElem>>
MetaTMPLowLevelSearch::
ValidMetaNeighbors(MetaElem _current, SemanticTask* _task, size_t _vid, double _currentCost, double _edgeCost) {

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	this->TMPLowLevelSearch::m_usedAgents[_current[_task]] = m_usedAgents[_current];

	auto currentCost = m_distance[_current][_task];
	
	auto elems = this->TMPLowLevelSearch::ValidNeighbors(_current[_task],_vid,currentCost,_edgeCost,true);

	std::vector<std::pair<double,MetaElem>> metas;

	auto meta = _current;
	for(auto elem : elems) {
		meta[_task] = elem.second;

		bool agentInUse = false;
		for(auto kv : _current) {
			if(kv.first == _task)
				continue;
			if(kv.second.m_agent == m_tmpLibrary->GetTaskPlan()->GetCoordinator())
				continue;
			if(kv.second.m_agent == elem.second.m_agent) {
				agentInUse = true;
			}
		}
		if(agentInUse)
			continue;

		double cost = _currentCost - m_distance[_current][_task] + elem.first;
		if(m_seen.count(meta)) {
			/// TODO::THIS IS ASSUMING A SOC METRIC-----NEED TO MAKE THIS FLEXIBLE ACROSS ALL GENERALCBS STUFF

			double oldCost = 0;
			for(auto kv : m_distance[meta]) {
				oldCost += kv.second;
			}
			if(cost >= oldCost)
				continue;
		}
		//if(!m_seen.count(meta) or oldCost > cost) {
			metas.push_back(std::make_pair(cost,meta));
			m_seen.insert(meta);
			m_parentMap[meta] = _current;

			m_distance[meta] = m_distance[_current];
			m_distance[meta][_task] = elem.first;

			m_execPathMap[meta] = this->TMPLowLevelSearch::m_execPathMap[elem.second];
			m_setupPathMap[meta] = this->TMPLowLevelSearch::m_setupPathMap[elem.second];

			bool exists = false;
			for(auto kv : m_reExecPathMap) {
				if(MetaElemCompare()(kv.first,_current) == MetaElemCompare()(_current,kv.first)) {
					exists = true;
					break;
				}
			}
			if(!exists) {
				m_reExecPathMap[_current] = {};
			}

			m_reExecPathMap[_current][meta] = this->TMPLowLevelSearch::m_reExecPathMap[_current[_task]][elem.second];

			m_usedAgents[meta] = m_usedAgents[_current];
			auto cfg = sg->GetGraph()->GetVertex(elem.second.m_vid);
			cfg.SetRobot(elem.second.m_agent->GetRobot());
			auto lastUse = std::make_pair(elem.first,cfg);
			m_usedAgents[meta][elem.second.m_agent] = lastUse;

			if(m_debug) {
				std::cout << "Adding transition: " << std::endl;
				std::cout << "Task: " 
									<< _task 
									<< " -> "
									<< _task->GetLabel()
									<< "\tAgent: " 
									<< elem.second.m_agent->GetRobot()->GetLabel()
									<< "\n\tVertices: "
									<< _current[_task].m_vid
									<< " -> "
									<< elem.second.m_vid
									<< "\nCost: "
									<< cost
									<< std::endl;
			}

		//}
	}

	return metas;
}

void
MetaTMPLowLevelSearch::
UpdateSolution(GeneralCBSNode& _node, std::vector<MetaElem> _plan) {
	if(m_debug) {
		for(auto meta : _plan) {
			for(auto elem : meta) {
				std::cout << "Task: " 
									<< elem.first->GetLabel()
									<< "\n\tAgent: "
									<< elem.second.m_agent->GetRobot()->GetLabel()
									<< "\n\tVID: "
									<< elem.second.m_vid
									<< std::endl;
			}
			std::cout << std::endl << std::endl;
		}
	}

	if(_plan.empty()) {
		throw RunTimeException(WHERE) << "Uncaught invalid plan."
																	<< std::endl;
	}

	std::unordered_map<SemanticTask*,std::vector<AvailElem>> individualPlans;

	// Add the virtual start elem to each individual plan
	for(auto kv : _plan[0]) {
		individualPlans[kv.first] = {_plan[0][kv.first]};
	}

	auto previous = _plan[0];
	// Separate the meta plan into individual elems
	for(size_t i = 1; i < _plan.size(); i++) {
		auto current = _plan[i];
		for(auto kv : current) {
	
			if(kv.second != previous[kv.first]) {
				individualPlans[kv.first].push_back(kv.second);

				auto exec = m_execPathMap[current];
				this->TMPLowLevelSearch::m_execPathMap[kv.second] = exec;//m_execPathMap[current];

				auto setup = m_setupPathMap[current];
				this->TMPLowLevelSearch::m_setupPathMap[kv.second] = setup;//m_setupPathMap[current];

				auto reExec = m_reExecPathMap[current][previous];
				this->TMPLowLevelSearch::m_reExecPathMap[kv.second][individualPlans[kv.first].back()] = reExec;//m_reExecPathMap[current];

				auto distance = m_distance[current][kv.first];
				this->TMPLowLevelSearch::m_distance[kv.second] = distance;//m_distance[current][kv.first];

				break;
			}
		}
		previous = current;
	}

	for(auto kv : individualPlans) {
		for(auto elem : kv.second) {
			std::cout << "Task: " 
								<< kv.first->GetLabel()
								<< "\n\tAgent: "
								<< elem.m_agent->GetRobot()->GetLabel()
								<< "\n\tVID: "
								<< elem.m_vid
								<< std::endl;
		}
		std::cout << std::endl << std::endl;
	}

	std::unordered_map<SemanticTask*,std::vector<Assignment>> planDetails;

	for(auto kv : individualPlans) {
		planDetails[kv.first] = this->TMPLowLevelSearch::PlanDetails(kv.second,kv.first);
		if(planDetails[kv.first].empty()) {
			throw RunTimeException(WHERE) << "Uncaught invalid plan for "
																		<< kv.first->GetLabel()
																		<< "."
																		<< std::endl;
		}
		_node.UpdateTaskPlan(kv.first,planDetails[kv.first]);
	}

}
