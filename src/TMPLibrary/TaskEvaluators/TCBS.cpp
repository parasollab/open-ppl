#include "TCBS.h"

#include "Simulator/Simulation.h"

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"

#include "Utilities/SSSP.h"

TCBS::
TCBS() {
	this->SetName("TCBS");
}

TCBS::
TCBS(XMLNode& _node) : TaskEvaluatorMethod(_node) {
	this->SetName("TCBS");
	m_sgLabel = _node.Read("sgLabel", true, "", "State graph to use.");
	m_makespan = _node.Read("makespan",false,false,"Flag to use makespan vs sum of cost.");
	m_vcLabel = _node.Read("vcLabel",true,"","Validity checker to use when finding collisions.");

}

TCBS::
~TCBS() { }

bool 
TCBS::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan) {
	if(_wholeTasks.empty()) {
		_wholeTasks = this->GetTaskPlan()->GetWholeTasks();
	}

	size_t totalNodes = 1;
	size_t maxDepth = 0;
	size_t solutionDepth = 0;
	size_t nodesExplored = 1;

	std::shared_ptr<TaskPlan> savedPlan = nullptr;
	if(_plan) { 
		savedPlan = this->GetTaskPlan();
		this->GetTMPLibrary()->SetTaskPlan(_plan);
	}

	auto initialNode = std::shared_ptr<Node>(new Node(_wholeTasks));

	auto minNode = initialNode;
	size_t minCost = MAX_INT;

	auto tree = NewCBSTree<HandoffAgent, std::pair<size_t,std::pair<size_t,size_t>>>();

	auto nodes = Expand(initialNode, _wholeTasks);	
	for(auto node : nodes) {
		UpdatePlan(node);
		tree.Insert(node);
	}

	while(!tree.Empty()) {

		std::shared_ptr<Node> node = std::shared_ptr<Node>(static_cast<Node*>(tree.GetMinNode()));
		nodesExplored++;
		if(node->GetDepth() > maxDepth)
			maxDepth = node->GetDepth();
	
		if(m_debug) {
			std::cout << "Tree size : " << tree.Length() << std::endl;
			std::cout << "Node depth : " << node->GetDepth();
		}

		auto newNodes = FindConflict(node);

		if(!newNodes.empty()) {
			for(auto node : newNodes) {
				UpdatePlan(node);
				tree.Insert(node);
				totalNodes++;
			}
		}
		else if(node->GetAssignedTasks().size() == _wholeTasks.size()) {
			//update min node status
			if(node->GetCost(m_makespan) < minCost) {
				minCost = node->GetCost(m_makespan);
				minNode = node;
				solutionDepth = node->GetDepth();
			}
		}
		else {
			newNodes = Expand(node, _wholeTasks);
			for(auto node : newNodes) {
				UpdatePlan(node);
				tree.Insert(node);
				totalNodes++;
			}
		}
	}
	
	FinalizePlan(minNode.get());

	Simulation::GetStatClass()->SetStat("TotalNodes", totalNodes);
	Simulation::GetStatClass()->SetStat("MaxDepth", maxDepth);
	Simulation::GetStatClass()->SetStat("SolutionDepth", solutionDepth);
	Simulation::GetStatClass()->SetStat("NodesExplored", nodesExplored);
	Simulation::GetStatClass()->SetStat("PlanCost", minNode->GetDiscreteCost(m_makespan));

	if(savedPlan)
		this->GetTMPLibrary()->SetTaskPlan(_plan);
	return true;
}

std::vector<TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>*>
TCBS::
Expand(std::shared_ptr<Node> _node, std::vector<WholeTask*> _tasks) {
	
	std::vector<Node*> newNodes;

	auto assignedTasks = _node->GetAssignedTasks();
	for(auto task : _tasks) {
		if(!assignedTasks.count(task)) {//check if task has an allocation
			for(auto agent : this->GetTaskPlan()->GetTeam()) {
				Node* newNode = new Node(_node.get(),agent);
				newNode->AssignTask(task,agent);
				newNodes.push_back(newNode);
			}
		}
	}	

	return newNodes;
}

std::vector<TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>*>
TCBS::
SplitVertexConflict(std::shared_ptr<Node> _node, size_t _conflict,
							HandoffAgent* _firstAgent, HandoffAgent* _secondAgent) {

	std::vector<Node*> newNodes;
	
	auto path = _node->GetAgentEntirePath(_firstAgent);
	size_t vid;
	if(_conflict < path.size())
		vid = path[_conflict];
	else if(!path.empty()) {
		vid = path.back();
		auto conflict = new MotionConflict(_firstAgent, std::make_pair(_conflict,std::make_pair(vid,MAX_INT)));

		Node* newNode = new Node(_node.get(),_firstAgent);
		newNode->AddConflict(_firstAgent, conflict);
		newNodes.push_back(newNode);
	}

	path = _node->GetAgentEntirePath(_secondAgent);
	if(_conflict < path.size())
		vid = path[_conflict];
	else if(!path.empty()){
		vid = path.back();
		auto conflict = new MotionConflict(_secondAgent, std::make_pair(_conflict,std::make_pair(vid,MAX_INT)));
	
		auto newNode = new Node(_node.get(), _secondAgent);
		newNode->AddConflict(_secondAgent, conflict);
		newNodes.push_back(newNode);
	}

	return newNodes;
}
		
std::vector<TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>*>
TCBS::
SplitEdgeConflict(std::shared_ptr<Node> _node, size_t _conflict,
							HandoffAgent* _firstAgent, HandoffAgent* _secondAgent) {

	std::vector<Node*> newNodes;
	
	auto path = _node->GetAgentEntirePath(_firstAgent);
	size_t vid1;
	size_t vid2 = MAX_INT;
	if(_conflict < path.size()-1) {
		vid1 = path[_conflict];
		vid2 = path[_conflict+1];
	}
	else {
		vid1 = path.back();
	}
	auto conflict = new MotionConflict(_firstAgent, std::make_pair(_conflict,std::make_pair(vid1,vid2)));

	Node* newNode = new Node(_node.get(),_firstAgent);
	newNode->AddConflict(_firstAgent, conflict);
	newNodes.push_back(newNode);

	vid2 = MAX_INT;

	path = _node->GetAgentEntirePath(_secondAgent);
	if(_conflict < path.size()-1) {
		vid1 = path[_conflict];
		vid2 = path[_conflict+1];
	}
	else {
		vid1 = path.back();
	}
	conflict = new MotionConflict(_secondAgent, std::make_pair(_conflict,std::make_pair(vid1,vid2)));
	
	newNode = new Node(_node.get(), _secondAgent);
	newNode->AddConflict(_secondAgent, conflict);
	newNodes.push_back(newNode);

	return newNodes;
}
 
std::vector<TCBSNode<HandoffAgent,std::pair<size_t,std::pair<size_t,size_t>>>*>
TCBS::
FindConflict(std::shared_ptr<Node> _node) {

	auto& team = this->GetTaskPlan()->GetTeam();

	std::unordered_map<HandoffAgent*,Cfg> originalPositions;
	for(auto agent : team) {
		originalPositions[agent] = agent->GetRobot()->GetSimulationModel()->GetState();
	}

	auto basevc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel).get();
	auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(basevc);

	for(size_t i = 0; i < team.size(); i++) {
		for(size_t j = i+1; j < team.size(); j++) {
			auto agent1 = team[i];
			auto agent2 = team[j];
			auto path1 = _node->GetAgentEntirePath(agent1);	
			auto path2 = _node->GetAgentEntirePath(agent2);
			
			auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
			auto roadmap1 = sg->GetCapabilityRoadmap(agent1);
			auto roadmap2 = sg->GetCapabilityRoadmap(agent2);

			size_t k = 0;
			Cfg cfg1 = originalPositions[agent1];
			Cfg cfg2 = originalPositions[agent2];
			Cfg cfg1b = cfg1;
			Cfg cfg2b = cfg2;
			for(; k < std::max(path1.size(), path2.size()); k++) {
				if(path1.size() > k) 
					cfg1 = roadmap1->GetVertex(path1[k]);	
				cfg1.SetRobot(agent1->GetRobot());

				if(path2.size() > k)
					cfg2 = roadmap2->GetVertex(path2[k]);
				cfg2.SetRobot(agent2->GetRobot());
				
				auto mb1 = cfg1.GetRobot()->GetMultiBody();
				mb1->Configure(cfg1);

				auto mb2 = cfg2.GetRobot()->GetMultiBody();
				mb2->Configure(cfg2);

				CDInfo cdInfo;
				//check vertex conflict
				if(vc->IsMultiBodyCollision(cdInfo, mb1, mb2, "Discrete cbs collision check.")) {
					for(auto agent : team) {
						originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
					}
					return SplitVertexConflict(_node, k, agent1, agent2);
				}
				//check edge conflicts
				if(!path1.empty() and path1.size()-1 > k) 
					cfg1b = roadmap1->GetVertex(path1[k+1]);	
				cfg1b.SetRobot(agent1->GetRobot());

				if(!path2.empty() and path2.size()-1 > k)
					cfg2b = roadmap2->GetVertex(path2[k+1]);
				cfg2b.SetRobot(agent2->GetRobot());
					
				mb1->Configure(cfg1b);
			
					
				if(vc->IsMultiBodyCollision(cdInfo, mb1, mb2, "Discrete cbs collision check.")) {
					for(auto agent : team) {
						originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
					}
					//return SplitEdgeConflict(_node, k, agent1, agent2);
					mb1->Configure(cfg1);
					mb2->Configure(cfg2b);
					if(vc->IsMultiBodyCollision(cdInfo, mb1, mb2, "Discrete cbs collision check.")) {
						for(auto agent : team) {
							originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
						}
						return SplitEdgeConflict(_node, k, agent1, agent2);
					}
				}

			}
			
			
		}
	}
	for(auto agent : team) {
		originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
	}
	return {};		
}

std::vector<size_t>
TCBS::
DiscreteSearch(Node* _node, size_t _start, size_t _goal) {
	size_t maxConstraint = 0;

	HandoffAgent* agent = _node->GetToReplan();
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(agent);

	//timestep to set of invalid vertices
	std::unordered_map<size_t,std::set<std::pair<size_t,size_t>>> pathConstraints;
	auto conflicts = _node->GetConflicts();
	for(auto constraint : (*conflicts)[agent]) {
		auto timeConstraint = constraint->GetConstraint();	
		pathConstraints[timeConstraint.first].insert(timeConstraint.second);
		if(timeConstraint.first > maxConstraint) {
			maxConstraint = timeConstraint.first;
		}
	}

	struct Vertex {
		size_t m_vid;
		size_t m_distance{0};
		Vertex* m_parent{nullptr};

		Vertex(size_t _vid, size_t _distance, Vertex* _parent) 
			: m_vid(_vid), m_distance(_distance), m_parent(_parent) {}
	};

	Vertex* current = new Vertex(_start,0,nullptr);

	std::list<Vertex*> pq;
	pq.push_back(current);

	while(current->m_vid != _goal) {
	
		if(current->m_distance > roadmap->Size() + maxConstraint)
			return {};
	
		auto vit = roadmap->find_vertex(current->m_vid);
		
		auto constraints = pathConstraints[current->m_distance];
		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			auto source = eit->source();
			auto target = eit->target();
			if(constraints.count(std::make_pair(target,MAX_INT))
			or constraints.count(std::make_pair(source, target))) {
				continue;
			}
			auto vert = new Vertex(target, current->m_distance+1, current);
			
			/*if(pq.empty())
				pq.push_back(vert);
			else 
				for(auto iter = pq.begin(); iter != pq.end(); iter++) {
					
				}
			*/
			//Just going to push back since each will increment by one each time
			pq.push_back(vert);
		}
		if(!constraints.count(std::make_pair(current->m_vid,MAX_INT))) {
			auto vert = new Vertex(current->m_vid,current->m_distance+1,current);
			pq.push_back(vert);
		}

		current = pq.front();
		pq.pop_front();
	}

	std::vector<size_t> path;
	while(current->m_vid != _start) {
		path.push_back(current->m_vid);
		current = current->m_parent;
	}
	path.push_back(current->m_vid);

	std::reverse(path.begin(), path.end());

	return path;
}

void
TCBS::
UpdatePlan(Node* _node) {
	auto agent = _node->GetToReplan();

	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(agent);

	auto start = agent->GetRobot()->GetSimulationModel()->GetState();
	int x = int(start[0]+.5);
	int y = int(start[1]+.5);
	start.SetData({double(x),double(y),0});
	start.SetRobot(this->GetTaskPlan()->GetDummyMap()[agent->GetCapability()]->GetRobot());

	size_t originVID = roadmap->GetVID(start);	

	auto tasks = _node->GetAgentAllocations(agent);
	for(auto task : tasks) {

		auto startCfg = task->m_startPoints[agent->GetCapability()][0];
		auto goalCfg = task->m_goalPoints[agent->GetCapability()][0];

		auto startVID = roadmap->GetVID(startCfg);
		auto goalVID = roadmap->GetVID(goalCfg);
		
		//Move to start
		auto setup = DiscreteSearch(_node, originVID, startVID);	
		if(m_debug) {
			std::cout << "Printing setup path." << std::endl;
			for(auto vid : setup) {
				auto cfg = roadmap->GetVertex(vid);
				std::cout << cfg.PrettyPrint() << std::endl;
			}
		}		

		//Execute Task
		auto execute = DiscreteSearch(_node, startVID, goalVID);
		if(m_debug) {
			std::cout << "Printing execution path." << std::endl;
			for(auto vid : execute) {
				auto cfg = roadmap->GetVertex(vid);
				std::cout << cfg.PrettyPrint() << std::endl;
			}
		}		

		if(setup.empty() or execute.empty()) {
			_node->DeallocateTask(agent,task);
			continue;
		}

		auto path = setup;
		for(size_t i = 1; i < execute.size(); i++) {
			path.push_back(execute[i]);
		}
		_node->AddAgentPath(agent, path);
	}
}

void
TCBS::
FinalizePlan(Node* _node) {
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
	for(auto agent : this->GetTaskPlan()->GetTeam()) {
		auto roadmap = sg->GetCapabilityRoadmap(agent);
		std::vector<Cfg> path;
		for(auto vid : _node->GetAgentEntirePath(agent)) {
			auto cfg = roadmap->GetVertex(vid);
			cfg.SetRobot(agent->GetRobot());
			path.push_back(cfg);
		}
		agent->SetPlan(path);
	}
}
