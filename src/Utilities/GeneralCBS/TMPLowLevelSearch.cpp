#include "TMPLowLevelSearch.h"

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

	Initialize(_node, _task);

	//auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());
	//TODO::Update MultiTaskGraph to take in SemanticTask
	//sg->AddTaskToGraph(_task->GetMotionTask());

	//TODO::find start and goal VIDs
	size_t start;
	size_t goal;
	
	auto plan = Search(start,goal);

	Uninitialize();

	if(plan.empty())
		return false;

	//TODO::change solution object within node

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
		for(auto agent : team) {
			if(cfg.GetRobot()->GetCapability() == agent->GetRobot()->GetCapability()) {
				m_intervalMap[vid][agent] = ComputeIntervals(_node, vid, agent);
			}
		}
	}
}

std::vector<std::pair<double,double>>
TMPLowLevelSearch::
ComputeIntervals(GeneralCBSNode& _node, size_t vid, Agent* _agent) {
	//need to map interval to departure time/location pairing while finding avail intervals
	return {};
}

void
TMPLowLevelSearch::
Uninitialize() {
	m_parentMap.clear();
	m_intervalMap.clear();
}

std::vector<Assignment>
TMPLowLevelSearch::
Search(size_t _start, size_t _goal) {

	//TODO::check for arbitrary start == goal path

	auto sg = static_cast<MultiTaskGraph*>(m_tmpLibrary->GetStateGraph(m_sgLabel).get());

	std::priority_queue<std::pair<double,AvailElem>> pq;
	std::set<AvailElem> visited;

	for(auto agentAvailInts : m_intervalMap[_start]) {
		for(auto avail : agentAvailInts.second) {
			AvailElem elem(_start, agentAvailInts.first, avail);
			pq.push(std::make_pair(0,elem));
		}
	}

	auto current = pq.top();
	pq.pop();

	do {

		auto vit = sg->GetGraph()->find_vertex(current.second.m_vid);
		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			auto elems = ValidNeighbors(current.second, eit->target(), current.first, eit->property().GetWeight());
			for(auto elem : elems) {
				pq.push(elem);
				m_parentMap[elem.second] = current.second;
			} 
		}

		current = pq.top();
		m_distance[current.second] = current.first;
		pq.pop();

	} while(!pq.empty() and current.second.m_vid != _goal);

	if(current.second.m_vid != _goal)
		return {};

	vector<AvailElem> plan;

	auto elem = current.second;

	do {
		elem = m_parentMap[elem];
		plan.push_back(elem);
	} while(elem.m_vid != _start);

	std::reverse(plan.begin(),plan.end());

	return PlanDetails(plan);

}
	
std::vector<std::pair<double,TMPLowLevelSearch::AvailElem>> 
TMPLowLevelSearch::
ValidNeighbors(const AvailElem& _elem, size_t _vid, double _currentCost, double _edgeCost) {
	return {};
}

std::pair<double,std::vector<size_t>>
TMPLowLevelSearch::
ComputeSetup(Agent* _agent, AvailInterval _avail, double _minTime, size_t _endVID) {	
	vector<size_t> path;
	return std::make_pair(0,path);
}

std::pair<double,std::vector<size_t>>
TMPLowLevelSearch::
ComputeExec(AvailElem& _elem, size_t _endVID) {
	vector<size_t> path;
	return std::make_pair(0,path);
}

std::vector<Assignment> 
TMPLowLevelSearch::
PlanDetails(std::vector<AvailElem> _plan) {
	return {};
}

Assignment 
TMPLowLevelSearch::
CreateAssignment(AvailElem _start, AvailElem _end) {
	return Assignment();
}
