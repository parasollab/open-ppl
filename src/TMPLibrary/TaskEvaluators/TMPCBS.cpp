#include "TMPCBS.h"

#include "Simulator/Simulation.h"

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

#include "TMPLibrary/StateGraphs/DiscreteIntervalGraph.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TMPTools/TMPTools.h"

#include "Utilities/SSSP.h"

TMPCBS::
TMPCBS() {
	this->SetName("TMPCBS");
}

TMPCBS::
TMPCBS(XMLNode& _node) : TaskEvaluatorMethod(_node) {
	this->SetName("TMPCBS");
	m_sgLabel = _node.Read("sgLabel", true, "", "State graph to use.");
	m_makespan = _node.Read("makespan",false,false,"Flag to use makespan vs sum of cost.");
	m_vcLabel = _node.Read("vcLabel",true,"","Validity checker to use when finding collisions.");
	m_dmadLabel = _node.Read("dmadLabel",true,"","Discrete MAD method to use.");
}

TMPCBS::
~TMPCBS() { }

bool 
TMPCBS::
Run(std::vector<WholeTask*> _wholeTasks, std::shared_ptr<TaskPlan> _plan) {
	if(_wholeTasks.empty()) {
		_wholeTasks = this->GetTaskPlan()->GetWholeTasks();
	}
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	size_t totalNodes = 1;
	size_t maxDepth = 0;
	size_t solutionDepth = 0;
	size_t nodesExplored = 1;

	std::shared_ptr<TaskPlan> savedPlan = nullptr;
	if(_plan) { 
		savedPlan = this->GetTaskPlan();
		this->GetTMPLibrary()->SetTaskPlan(_plan);
	}

	for(auto member : this->GetTaskPlan()->GetTeam()) {
		auto cfg = member->GetRobot()->GetSimulationModel()->GetState();
		int x = int(cfg[0]+.5);
		int y = int(cfg[1]+.5);
		cfg.SetData({double(x),double(y),0});
		cfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(member->GetCapability())->GetRobot());
		auto vid = sg->GetCapabilityRoadmap(member)->GetVID(cfg);
		DiscreteAgentAllocation allocation(member, 0, 0, vid, vid);
		this->GetTaskPlan()->AddAgentAllocation(member, allocation);
	}


	auto initialNode = std::shared_ptr<Node>(new Node(_wholeTasks));

	auto minNode = initialNode;
	size_t minCost = MAX_INT;

	auto tree = NewCBSTree<WholeTask, std::pair<size_t,std::pair<size_t,size_t>>>();
	
	auto ag = sg->GetAvailableIntervalGraph();
	sg->PrintGraph();

	for(auto task : _wholeTasks) {
		initialNode->SetToReplan(task);
		UpdatePlan(initialNode.get());
	}

	auto& initialVIDs = initialNode->GetValidVIDs();
	
	for(auto kv : sg->GetTaskAigVIDs()) {
		for(auto vid : kv.second) {
			initialVIDs[kv.first].insert(vid);
		}
		for(auto vit = sg->GetAvailableIntervalGraph()->begin();
				vit != sg->GetAvailableIntervalGraph()->end(); vit++) {
			if(!sg->GetAvailableIntervalGraph()->IsVertexInvalidated(vit->descriptor()))
				initialVIDs[kv.first].insert(vit->descriptor());
		}
	}

	/*
	auto motionConflict = new MotionConflict(this->GetTaskPlan()->GetTeam()[0],
																								   std::make_pair(1,std::make_pair(7,MAX_INT)));
	initialNode->AddMotionConflict(_wholeTasks[0], this->GetTaskPlan()->GetTeam()[0], motionConflict);

	motionConflict = new MotionConflict(this->GetTaskPlan()->GetTeam()[0],
																								   std::make_pair(3,std::make_pair(7,MAX_INT)));
	initialNode->AddMotionConflict(_wholeTasks[0], this->GetTaskPlan()->GetTeam()[0], motionConflict);
	motionConflict = new MotionConflict(this->GetTaskPlan()->GetTeam()[0],
																								   std::make_pair(2,std::make_pair(7,MAX_INT)));
	initialNode->AddMotionConflict(_wholeTasks[0], this->GetTaskPlan()->GetTeam()[0], motionConflict);
	std::cout << "Updating motion constraint." << std::endl;
	
	auto validVIDs = sg->UpdateMotionConstraint(this->GetTaskPlan()->GetTeam()[0],_wholeTasks[0],  
														 initialNode->GetValidVIDs()[_wholeTasks[0]], 
														 initialNode->GetTaskMotionConstraints(_wholeTasks[0]));

	initialNode->UpdateValidVIDs(validVIDs.first,validVIDs.second,_wholeTasks[0]);
	*/
	//auto constraintMap = initialNode->GetTaskMotionConstraints(_wholeTasks[0]);
	//auto validVIDs = sg->UpdateAvailableIntervalConstraint(this->GetTaskPlan()->GetTeam()[0],0,12,4,4, _wholeTasks[0],
	//																			initialNode->GetValidVIDs()[_wholeTasks[0]], constraintMap);

	//initialNode->UpdateValidVIDs(validVIDs.first,validVIDs.second, _wholeTasks[0]);
	//sg->UpdateAvailableIntervalConstraint(this->GetTaskPlan()->GetTeam()[0],17,27,4,4, _wholeTasks[0],
	//																			initialNode->GetValidVIDs()[_wholeTasks[0]], constraintMap);
	//std::cout << "Just updated graph." << std::endl;
	//sg->PrintGraph();
	//sg->PrintAvailabilityGraph();
	auto nodes = FindConflict(initialNode);
	for(auto node : nodes) {
		if(UpdatePlan(node)) {
			tree.Insert(node);
			totalNodes++;
		}
	}	

	while(!tree.Empty()) {

		std::shared_ptr<Node> node = std::shared_ptr<Node>(static_cast<Node*>(tree.GetMinNode()));

		size_t nodeCost = node->GetDiscreteCost(m_makespan);

		if(nodeCost > minCost)
			break;

		nodesExplored++;
		if(node->GetDepth() > maxDepth)
			maxDepth = node->GetDepth();
	
		if(m_debug) {
			std::cout << "Tree size : " << tree.Length() << std::endl;
			std::cout << "Node depth : " << node->GetDepth() << std::endl;
		}

		auto newNodes = FindConflict(node);

		if(!newNodes.empty()) {
			for(auto newNode : newNodes) {
				if(UpdatePlan(newNode)) {
					tree.Insert(newNode);
					totalNodes++;
				}
			}
		}
		else if(nodeCost < minCost) {
			minCost = nodeCost;
			minNode = node;
			solutionDepth = node->GetDepth();
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

std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
SplitVertexConflict(std::shared_ptr<Node> _node, WholeTask* _task1, WholeTask* _task2,
							HandoffAgent* _agent1, HandoffAgent* _agent2, size_t _time, size_t _vid1, size_t _vid2) {
	
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	std::vector<Node*> newNodes;

	auto motion1 = std::make_pair(_time,std::make_pair(_vid1,MAX_INT));
	auto conflict1 = new MotionConflict(_agent1, motion1);

	Node* newNode = new Node(_node.get(),_task1);
	newNode->AddMotionConflict(_task1,_agent1,conflict1);

	auto validVIDs = sg->UpdateMotionConstraint(_agent1,_task1,  
														 newNode->GetValidVIDs()[_task1], 
														 newNode->GetTaskMotionConstraints(_task1));
	newNode->UpdateValidVIDs(validVIDs.first, validVIDs.second,_task1);

	newNodes.push_back(newNode);

	auto motion2 = std::make_pair(_time,std::make_pair(_vid2,MAX_INT));
	auto conflict2 = new MotionConflict(_agent2, motion2);

	newNode = new Node(_node.get(),_task2);
	newNode->AddMotionConflict(_task2,_agent2,conflict2);

	validVIDs = sg->UpdateMotionConstraint(_agent2,_task2,  
														 newNode->GetValidVIDs()[_task2], 
														 newNode->GetTaskMotionConstraints(_task2));
	newNode->UpdateValidVIDs(validVIDs.first,validVIDs.second,_task2);

	newNodes.push_back(newNode);

	/*
	auto path = _node->GetAgentEntirePath(_firstAgent);
	size_t vid;
	if(_conflict < path.size() and !path.empty()) {
		vid = path[_conflict];
	//else if(!path.empty())  
		//vid = path.back();
	//if(!path.empty()) {
		auto conflict = new MotionConflict(_firstAgent, std::make_pair(_conflict,std::make_pair(vid,MAX_INT)));

		Node* newNode = new Node(_node.get(),_firstAgent);
		newNode->AddConflict(_firstAgent, conflict);
		newNodes.push_back(newNode);
	}

	path = _node->GetAgentEntirePath(_secondAgent);
	if(_conflict < path.size() and !path.empty()) { 
		vid = path[_conflict];
	//else if(!path.empty())
		//vid = path.back();
	//if(!path.empty()) {
		auto conflict = new MotionConflict(_secondAgent, std::make_pair(_conflict,std::make_pair(vid,MAX_INT)));
	
		auto newNode = new Node(_node.get(), _secondAgent);
		newNode->AddConflict(_secondAgent, conflict);
		newNodes.push_back(newNode);
	}
	*/
	return newNodes;
}
		
std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
SplitEdgeConflict(std::shared_ptr<Node> _node, WholeTask* _task1, WholeTask* _task2, HandoffAgent* _agent1, HandoffAgent* _agent2,
									size_t _time, size_t _vid1, size_t _vid1b, size_t _vid2, size_t _vid2b) {

	std::vector<Node*> newNodes;

	
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	auto motion1 = std::make_pair(_time,std::make_pair(_vid1,_vid1b));
	auto conflict1 = new MotionConflict(_agent1, motion1);

	Node* newNode = new Node(_node.get(),_task1);
	newNode->AddMotionConflict(_task1,_agent1,conflict1);

	auto validVIDs = sg->UpdateMotionConstraint(_agent1,_task1,  
														 newNode->GetValidVIDs()[_task1], 
														 newNode->GetTaskMotionConstraints(_task1));
	newNode->UpdateValidVIDs(validVIDs.first, validVIDs.second,_task1);

	newNodes.push_back(newNode);

	auto motion2 = std::make_pair(_time,std::make_pair(_vid2,_vid2b));
	auto conflict2 = new MotionConflict(_agent2, motion2);

	newNode = new Node(_node.get(),_task2);
	newNode->AddMotionConflict(_task2,_agent2,conflict2);

	validVIDs = sg->UpdateMotionConstraint(_agent2,_task2,  
														 newNode->GetValidVIDs()[_task2], 
														 newNode->GetTaskMotionConstraints(_task2));
	newNode->UpdateValidVIDs(validVIDs.first,validVIDs.second,_task2);

	newNodes.push_back(newNode);
	/*
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
	*/
	return newNodes;
}

std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
FindTaskConflicts(std::shared_ptr<Node> _node) {
	std::vector<Node*> newNodes;
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	auto tasks = this->GetTaskPlan()->GetWholeTasks();
	for(size_t i = 0; i < tasks.size(); i++) {
		auto plan1 = _node->GetTaskPlan(tasks[i]);
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto plan2 = _node->GetTaskPlan(tasks[j]);

			auto iter1 = plan1.begin();
			auto iter2 = plan2.begin();

			while(iter1 != plan1.end() and iter2 != plan2.end()) {
				//No agent overlap. Iterate first ending subtask
				if(iter1->m_agent != iter2->m_agent) {
					if(iter1->m_subtaskStartTime+iter1->m_subtaskPath.size() 
							< iter2->m_subtaskStartTime+iter2->m_subtaskPath.size())
						iter1++;
					else 
						iter2++;
					continue;
				}
				//Subtask 1 starts after subtask 2 is complete
				if(iter1->m_setupStartTime > iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()) {
					iter2++;
					continue;
				}
				else if(iter2->m_setupStartTime > iter1->m_subtaskStartTime + iter1->m_subtaskPath.size()) {
					iter1++;
					continue;
				}
				else {
					auto newNode1 = new Node(_node.get(),tasks[i]);
					DiscreteAgentAllocation alloc1(iter2->m_agent, iter2->m_subtaskStartTime,
																				 iter2->m_subtaskStartTime + iter2->m_subtaskPath.size(), 
																				 iter2->m_subtaskPath[0],
																				 iter2->m_subtaskPath.back());
					auto taskConflict1 = new TaskConflict(tasks[i],alloc1);
					newNode1->AddTaskConflict(tasks[i],taskConflict1);

					auto validVIDs = sg->UpdateAvailableIntervalConstraint(iter2->m_agent,
																								iter2->m_subtaskStartTime,
																				 				iter2->m_subtaskStartTime + iter2->m_subtaskPath.size(), 
																				 				iter2->m_subtaskPath[0],
																				 				iter2->m_subtaskPath.back(),
																								tasks[i],
																								newNode1->GetValidVIDs()[tasks[i]], 
																								newNode1->GetTaskMotionConstraints(tasks[i]));
					newNode1->UpdateValidVIDs(validVIDs.first,validVIDs.second, tasks[i]);
					newNodes.push_back(newNode1);
	
					auto newNode2 = new Node(_node.get(),tasks[j]);
					DiscreteAgentAllocation alloc2(iter1->m_agent, iter1->m_subtaskStartTime,
																				 iter1->m_subtaskStartTime + iter1->m_subtaskPath.size(), 
																				 iter1->m_subtaskPath[0],
																				 iter1->m_subtaskPath.back());
					auto taskConflict2 = new TaskConflict(tasks[j],alloc2);
					newNode2->AddTaskConflict(tasks[j],taskConflict2);
					validVIDs = sg->UpdateAvailableIntervalConstraint(iter1->m_agent,
																								iter1->m_subtaskStartTime,
																				 				iter1->m_subtaskStartTime + iter2->m_subtaskPath.size(), 
																				 				iter1->m_subtaskPath[0],
																				 				iter1->m_subtaskPath.back(),
																								tasks[j],
																								newNode2->GetValidVIDs()[tasks[j]], 
																								newNode2->GetTaskMotionConstraints(tasks[j]));
					newNode2->UpdateValidVIDs(validVIDs.first,validVIDs.second, tasks[j]);
					newNodes.push_back(newNode2);
					return newNodes;	
				}
				//TODO::Check if setup is the issue and can be resolved without affecting subtask start
			}
		}
	}
	return newNodes;
}
 
std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
FindMotionConflicts(std::shared_ptr<Node> _node) {
	std::vector<Node*> newNodes;
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	
	
	auto tasks = this->GetTaskPlan()->GetWholeTasks();

	//check for conflicts between setup paths and executions paths within the same task
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto& subtasks = _node->GetTaskPlan(tasks[i]);
		for(size_t j = 1; j < subtasks.size(); j++) {
			auto plan1 = subtasks[j];//setup
			auto plan2 = subtasks[j-1];//exec
		
			auto& setupPath = plan1.m_setupPath;
			auto& executionPath = plan2.m_subtaskPath;

			auto& setupStartTime = plan1.m_setupStartTime;
			auto& execStartTime = plan2.m_subtaskStartTime;

			size_t setupOffset = 0;
			size_t execOffset = 0;
			if(execStartTime > setupStartTime) {
				setupOffset = execStartTime - setupStartTime;
			}
			else {
				execOffset = setupStartTime - execStartTime;
			}	

			auto roadmap1 = sg->GetCapabilityRoadmap(plan1.m_agent);
			auto roadmap2 = sg->GetCapabilityRoadmap(plan2.m_agent);

			while(setupOffset < setupPath.size() and execOffset < executionPath.size()) {
				auto sVID = setupPath[setupOffset];
				auto eVID = executionPath[execOffset];
				auto sCfg = roadmap1->GetVertex(sVID);
				auto eCfg = roadmap2->GetVertex(eVID);

				if(sCfg[0] == eCfg[0] and sCfg[1] == eCfg[1]) {
					return SplitVertexConflict(_node,tasks[i],tasks[i],plan1.m_agent,plan2.m_agent,setupOffset + plan1.m_setupStartTime,sVID,eVID);
				}
				
				setupOffset++;
				execOffset++;
				if(setupOffset == setupPath.size() or execOffset == executionPath.size())
					continue;

				auto nsVID = setupPath[setupOffset];
				auto neVID = executionPath[execOffset];
				auto nsCfg = roadmap1->GetVertex(sVID);
				auto neCfg = roadmap2->GetVertex(eVID);

				if(nsCfg[0] == neCfg[0] and sCfg[1] == eCfg[1]
				and sCfg[0] == eCfg[0] and nsCfg[1] == neCfg[1]) {
					return SplitEdgeConflict(_node,tasks[i],tasks[i],plan1.m_agent,plan2.m_agent,
																	 setupOffset + plan1.m_setupStartTime,sVID,nsVID,eVID,neVID);
				}
			}
		}
	}

	//check for conflicts between execution paths
	
	std::unordered_map<WholeTask*,std::vector<std::pair<HandoffAgent*,size_t>>> execPaths;

	for(auto task : tasks) {
		auto plan = _node->GetTaskPlan(task);
		size_t index = 0;
		execPaths[task] = {};
		for(auto subtask : plan) {
			if(subtask.m_subtaskStartTime >= index) {
				for(;index <= subtask.m_subtaskStartTime; index++) {
					execPaths[task].push_back(std::make_pair(nullptr,MAX_INT));
				}
			}
			for(auto vid : subtask.m_subtaskPath) {
				execPaths[task].push_back(std::make_pair(subtask.m_agent,vid));
				index++;
			}
		}
	}

	
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto path1 = execPaths[tasks[i]];
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto path2 = execPaths[tasks[j]];
			
			size_t index = 0;
			auto agent1 = path1[index].first;
			auto vid1 = path1[index].second;
			auto agent2 = path2[index].first;
			auto vid2 = path2[index].second;
			
			while(index < path1.size() and index < path2.size()) {
				if(!agent1 or !agent2) {
					index++;
					agent1 = path1[index].first;
					vid1 = path1[index].second;
					agent2 = path2[index].first;
					vid2 = path2[index].second;
					continue;
				}
			
				Cfg cfg1 = sg->GetCapabilityRoadmap(agent1)->GetVertex(vid1);
				Cfg cfg2 = sg->GetCapabilityRoadmap(agent2)->GetVertex(vid2);
				
				if(cfg1[0] == cfg2[0] and cfg1[1] == cfg2[1]) {
					return SplitVertexConflict(_node,tasks[i],tasks[j],agent1,agent2,index,vid1,vid2);
				}

				index++;
				if(index >= path1.size() or index >= path2.size())
					continue;
				
				agent1 = path1[index].first;
				auto vid1b = path1[index].second;
				agent2 = path2[index].first;
				auto vid2b = path2[index].second;
			
				Cfg cfg1b = sg->GetCapabilityRoadmap(agent1)->GetVertex(vid1b);
				Cfg cfg2b = sg->GetCapabilityRoadmap(agent2)->GetVertex(vid2b);

				if(cfg1[0] == cfg2b[0] and cfg1[1] == cfg2b[1]
				and cfg1b[0] == cfg2[0] and cfg1b[1] == cfg2[1]) {
					return SplitEdgeConflict(_node,tasks[i],tasks[j],agent1,agent2,
																	 index-1,vid1,vid1b,vid2,vid2b);
				}

				vid1 = vid1b;
				vid2 = vid2b;
			}
		}
	}

	//use an unordered_map instead of just a pair because there could be multiple agents moving
	//along multiple setup paths simultanesoulsy for a single task
	std::unordered_map<WholeTask*,std::vector<std::unordered_map<HandoffAgent*,size_t>>> setupPaths;

	for(auto task : tasks) {
		auto plan = _node->GetTaskPlan(task);
		setupPaths[task] = {};
		std::vector<std::vector<std::pair<HandoffAgent*,size_t>>> paths;
		size_t max = 0;
		for(auto subtask : plan) {
			size_t index = 0;
			std::vector<std::pair<HandoffAgent*,size_t>> setupPath;
			if(subtask.m_setupStartTime > index) {
				for(;index < subtask.m_subtaskStartTime; index++) {
					setupPath.push_back(std::make_pair(nullptr,MAX_INT));
				}
			}
			for(auto vid : subtask.m_setupPath) {
				setupPath.push_back(std::make_pair(subtask.m_agent,vid));
				index++;
			}
			if(index > max)
				max = index;
			paths.push_back(setupPath);
		}
		size_t index = 0;
		while(index < max) {
			std::unordered_map<HandoffAgent*,size_t> step;
			bool any = false;
			for(auto path : paths) {
				if(path[index].first) {
					any = true;
					step[path[index].first] = path[index].second;
				}
			}
			if(!any) {
				step[nullptr] = MAX_INT;
			}
			setupPaths[task].push_back(step);
			index++;
		}
	}
	//Check setup paths against execution paths
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto execPath = execPaths[tasks[i]];
		for(size_t j = 0; j < tasks.size(); j++) {
			if(i == j)
				continue;
			
			auto setupPath = setupPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(!execPath[index].first or setupPath[index].size() == 0) {
					index++;
					continue;
				}
				for(auto agentVID : setupPath[index]) {
					auto execAgent = execPath[index].first;
					auto execVID = execPath[index].second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					auto setupAgent = agentVID.first;
					auto setupVID = agentVID.second;	
					auto setupCfg = sg->GetCapabilityRoadmap(setupAgent)->GetVertex(setupVID);

					if(execCfg[0] == setupCfg[0] and execCfg[1] == setupCfg[1])
						return SplitVertexConflict(_node,tasks[i],tasks[j],execAgent,setupAgent,index,execVID,setupVID);

					//Check if paths continues and if so check for edge conflict
					if(index == execPath.size()-1 or index == setupPath.size()-1)
						continue;
					auto iter = setupPath[index+1].find(setupAgent);
					if(iter == setupPath[index+1].end())
						continue;

					auto execVIDb = execPath[index+1].second;
					auto setupVIDb = iter->second;

					auto execCfgb = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVIDb);
					auto setupCfgb = sg->GetCapabilityRoadmap(setupAgent)->GetVertex(setupVIDb);

					if(execCfg[0] == setupCfgb[0] and execCfg[1] == setupCfgb[1] 
					and execCfgb[0] == setupCfg[0] and execCfgb[1] == setupCfg[1]) { 
						return SplitEdgeConflict(_node,tasks[i],tasks[j],execAgent,setupAgent,
																	 index,execVID,execVIDb,setupVID,setupVIDb);

					}
					
				}
				index++;
			}
	
		}
	}
	//Check setup paths against setup paths
	//leaving first setup path labeled exec for now because it's late and im lazy
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto execPath = setupPaths[tasks[i]];
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto setupPath = setupPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(execPath[index].size() == 0 or setupPath[index].size() == 0) {
					index++;
					continue;
				}
				for(auto execAgentVID : execPath[index]) {
					auto execAgent = execAgentVID.first;
					auto execVID = execAgentVID.second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					for(auto agentVID : setupPath[index]) {
						auto setupAgent = agentVID.first;
						auto setupVID = agentVID.second;	
						auto setupCfg = sg->GetCapabilityRoadmap(setupAgent)->GetVertex(setupVID);

						if(execCfg[0] == setupCfg[0] and execCfg[1] == setupCfg[1])
							return SplitVertexConflict(_node,tasks[i],tasks[j],execAgent,setupAgent,index,execVID,setupVID);

						//Check if paths continue and if so check for edge conflict
						if(index == execPath.size()-1 or index == setupPath.size()-1)
							continue;
						auto execIter = execPath[index+1].find(execAgent);
						if(execIter == execPath[index+1].end())
							continue;

						auto iter = setupPath[index+1].find(setupAgent);
						if(iter == setupPath[index+1].end())
							continue;

						auto execVIDb = execIter->second;
						auto setupVIDb = iter->second;

						auto execCfgb = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVIDb);
						auto setupCfgb = sg->GetCapabilityRoadmap(setupAgent)->GetVertex(setupVIDb);

						if(execCfg[0] == setupCfgb[0] and execCfg[1] == setupCfgb[1] 
						and execCfgb[0] == setupCfg[0] and execCfgb[1] == setupCfg[1]) 
							return SplitEdgeConflict(_node,tasks[i],tasks[j],execAgent,setupAgent,
																	 index,execVID,execVIDb,setupVID,setupVIDb);
					
					}
				}
				index++;
			}
	
		}
	}
	return {};
}

std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
FindConflict(std::shared_ptr<Node> _node) {

	auto taskConflicts = FindTaskConflicts(_node);
	if(!taskConflicts.empty())
		return taskConflicts;

	return FindMotionConflicts(_node);
	/*
	auto& team = this->GetTaskPlan()->GetTeam();

	std::unordered_map<HandoffAgent*,Cfg> originalPositions;
	for(auto agent : team) {
		originalPositions[agent] = agent->GetRobot()->GetSimulationModel()->GetState();
	}

	//auto basevc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel).get();
	//auto vc = dynamic_cast<CollisionDetectionValidity<MPTraits<Cfg>>*>(basevc);
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

	for(size_t i = 0; i < team.size(); i++) {
		for(size_t j = i+1; j < team.size(); j++) {
			auto agent1 = team[i];
			auto agent2 = team[j];
			auto path1 = _node->GetAgentEntirePath(agent1);	
			auto path2 = _node->GetAgentEntirePath(agent2);
			
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
				
				//auto mb1 = cfg1.GetRobot()->GetMultiBody();
				//mb1->Configure(cfg1);

				//auto mb2 = cfg2.GetRobot()->GetMultiBody();
				//mb2->Configure(cfg2);

				//CDInfo cdInfo;
				//check vertex conflict
				//if(vc->IsMultiBodyCollision(cdInfo, mb1, mb2, "Discrete cbs collision check.")) {
				//	for(auto agent : team) {
				//		originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
				//	}
				//	return SplitVertexConflict(_node, k, agent1, agent2);
				//}
				if(cfg1[0] == cfg2[0] and cfg1[1] == cfg2[1]) {
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
					
				//mb1->Configure(cfg1b);
			
					
				if(cfg1b[0] == cfg2[0] and cfg1b[1] == cfg2[1] and cfg1[0] == cfg2b[0] and cfg1[1] == cfg2b[1]) {
					for(auto agent : team) {
						originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
					}
					return SplitEdgeConflict(_node, k, agent1, agent2);
				}
			}
		}
	}
	for(auto agent : team) {
		originalPositions[agent].GetRobot()->GetMultiBody()->Configure(originalPositions[agent]);				
	}*/
	return {};		
}

std::vector<size_t>
TMPCBS::
DiscreteSearch(Node* _node, size_t _start, size_t _goal, size_t _startTime, size_t _minEndTime) {
	/*size_t maxConstraint = 0;

	HandoffAgent* agent = _node->GetToReplan();
	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(agent);

	//timestep to set of invalid vertices
	std::unordered_map<size_t,std::set<std::pair<size_t,size_t>>> pathConstraints;
	auto conflicts = _node->GetConflicts();
	for(auto constraint : (*conflicts)[agent]) {
		auto timeConstraint = constraint->GetConstraint();	
		if(timeConstraint.second.second == MAX_INT) {
			pathConstraints[timeConstraint.first-1].insert(timeConstraint.second);
		}
		else {
			pathConstraints[timeConstraint.first].insert(timeConstraint.second);
		}
		if(timeConstraint.first > maxConstraint) {
			maxConstraint = timeConstraint.first;
		}
	}

	struct Vertex {
		size_t m_vid;
		size_t m_distance;
		std::shared_ptr<Vertex> m_parent{nullptr};

		Vertex(size_t _vid, size_t _distance, std::shared_ptr<Vertex> _parent) 
			: m_vid(_vid), m_distance(_distance), m_parent(_parent) {}
	};

	auto current = std::shared_ptr<Vertex>(new Vertex(_start,_startTime,nullptr));
	
	std::list<std::shared_ptr<Vertex>> pq;
	size_t currentDistance = _startTime+1;
	std::set<size_t> discoveredVertices;
	while(current->m_vid != _goal or current->m_distance < _minEndTime) {	

		if(current->m_distance > roadmap->Size() + maxConstraint)
			return {};
	
		if(current->m_distance == currentDistance) {
			currentDistance++;
			discoveredVertices.clear();
		}

		auto vit = roadmap->find_vertex(current->m_vid);
		
		auto constraints = pathConstraints[current->m_distance];
		if(!constraints.count(std::make_pair(current->m_vid,MAX_INT))) {
			auto vert = std::shared_ptr<Vertex>(new Vertex(current->m_vid,current->m_distance+1,current));
			pq.push_back(vert);
			discoveredVertices.insert(current->m_vid);
		}

		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			auto source = eit->source();
			auto target = eit->target();
			if(constraints.count(std::make_pair(target,MAX_INT))
			or constraints.count(std::make_pair(source, target))) {
				if(m_debug) {
					std::cout << "Constrained vertex/time: " << target << " : " << current->m_distance+1 << std::endl;
				}
				continue;
			}
			else if(target == _goal) {
				auto taskSwitchingConstraints = pathConstraints[current->m_distance+2];
				if(taskSwitchingConstraints.count(std::make_pair(target,MAX_INT))) {
					continue;
				}
			}
			if(!discoveredVertices.count(target)) {
				auto vert = std::shared_ptr<Vertex>(new Vertex(target, current->m_distance+1, current));
	*/		
				/*if(pq.empty())
					pq.push_back(vert);
				else 
					for(auto iter = pq.begin(); iter != pq.end(); iter++) {
					
					}
				*/
				//Just going to push back since each will increment by one each time
/*				pq.push_back(vert);
				discoveredVertices.insert(target);
			}
		}

		current = pq.front();
		pq.pop_front();
	}

	std::vector<size_t> path;
	while(current->m_vid != _start or current->m_distance != _startTime) {
		path.push_back(current->m_vid);
		current = current->m_parent;
	}
	path.push_back(current->m_vid);

	std::reverse(path.begin(), path.end());

	return path;
	*/
	return {};
}

bool
TMPCBS::
UpdatePlan(Node* _node) {

	auto task = _node->GetToReplan();
	
	auto plan = this->GetTMPTools()->GetDiscreteMAD(m_dmadLabel)->Run(task, _node->GetValidVIDs()[task],
																																		_node->GetTaskMotionConstraints(task));
	_node->SetTaskPlan(task,plan);

	return true;
}

void
TMPCBS::
FinalizePlan(Node* _node) {
	//auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	/*for(auto agent : this->GetTaskPlan()->GetTeam()) {
		auto roadmap = sg->GetCapabilityRoadmap(agent);
		std::vector<Cfg> path;
		for(auto vid : _node->GetAgentEntirePath(agent)) {
			auto cfg = roadmap->GetVertex(vid);
			cfg.SetRobot(agent->GetRobot());
			path.push_back(cfg);
		}
		agent->SetPlan(path);
	}*/
}

std::vector<size_t>
TMPCBS::
TaskSearch(Node* _node) {

	/*
	auto task = _node->GetToReplan();
	
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto query = sg->AddTaskToGraph(task, _node->GetValidVIDs()[task]);

	auto start = query.first;
	auto goal = query.second;

	std::unordered_map<size_t,std::unordered_map<Agent*,std::pair<size_t,Cfg>>> robotUpdates;

	for(auto agent : this->GetTaskPlan()->GetTeam()) {
		Cfg cfg(agent->GetRobot());
		robotUpdates[start][agent] = std::make_pair(0,cfg);
	}	
	*/

	return {};
}
