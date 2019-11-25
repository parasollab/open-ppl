#include "TMPCBS.h"

#include "Simulator/Simulation.h"

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

#include "TMPLibrary/StateGraphs/DiscreteIntervalGraph.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TMPTools/TMPTools.h"

#include "Utilities/GeneralCBS/AllocationValidation.h"
#include "Utilities/GeneralCBS/ComposeValidation.h"
#include "Utilities/GeneralCBS/GeneralCBS.h"
#include "Utilities/GeneralCBS/MotionValidation.h"
#include "Utilities/GeneralCBS/TMPLowLevelSearch.h"
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

	Simulation::GetStatClass()->StartClock("PlanningTime");	

	if(_wholeTasks.empty()) {
		_wholeTasks = this->GetTaskPlan()->GetWholeTasks();
	}

	std::shared_ptr<TaskPlan> savedPlan = nullptr;
	if(_plan) { 
		savedPlan = this->GetTaskPlan();
		this->GetTMPLibrary()->SetTaskPlan(_plan);
	}

	auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

	sg->PrintGraph();

	//TODO::Create decomposition out of task plan
	auto decomp = CreateDecomposition(_wholeTasks);
	//auto decomp = new Decomposition(std::shared_ptr<SemanticTask>(new SemanticTask()));
	
	TMPLowLevelSearch lowLevel(this->GetTMPLibrary(), m_sgLabel);

	auto alloc = new AllocationValidation(this->GetMPLibrary(), &lowLevel, this->GetTMPLibrary());
	auto motion = new MotionValidation(this->GetMPLibrary(), &lowLevel);
	auto compose = new ComposeValidation({alloc, motion});

	InitialPlanFunction init = [this,compose](Decomposition* _decomp, GeneralCBSTree& _tree) {
		return compose->InitialPlan(_decomp, _tree);
	};

	ValidationFunction valid = [this,compose](GeneralCBSNode& _node, GeneralCBSTree& _tree) {
		return compose->ValidatePlan(_node, _tree);
	};

	CBSSolution solution = ConflictBasedSearch(decomp, init, valid);	

/*
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
*/

	//initially commented iout
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
	

	//end of intially commented out
	/*
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

		if(nodeCost >= minCost)
			break;

		nodesExplored++;
		if(node->GetDepth() > maxDepth)
			maxDepth = node->GetDepth();

		if(nodesExplored > 100000)
			break;
	
		if(m_debug) {
			std::cout << "Tree size : " << tree.Length() << std::endl;
			std::cout << "Node depth : " << node->GetDepth() << std::endl;
		}

		if(tree.Length() == 57 and node->GetDepth() == 9)
			std::cout << "HERE" << std::endl;

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
	
	Simulation::GetStatClass()->StopClock("PlanningTime");	
	FinalizePlan(minNode.get());

	Simulation::GetStatClass()->SetStat("TotalNodes", totalNodes);
	Simulation::GetStatClass()->SetStat("MaxDepth", maxDepth);
	Simulation::GetStatClass()->SetStat("SolutionDepth", solutionDepth);
	Simulation::GetStatClass()->SetStat("NodesExplored", nodesExplored);
	Simulation::GetStatClass()->SetStat("Makespan", minNode->GetDiscreteCost(true));
	Simulation::GetStatClass()->SetStat("SOC", minNode->GetDiscreteCost(false));
  Simulation::Get()->PrintStatFile();

	*/
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


	if(m_debug) {
		std::cout << "Split Vertex" << std::endl;
		std::cout << "Node: " << _node << std::endl;
		std::cout << "Child: " << newNodes[0] << " Task: " << _task1 << std::endl;
		std::cout << "Robot: " << _agent1->GetRobot()->GetLabel() << std::endl;
		std::cout << "Time: " << _time << " Edge: " << _vid1 << " -> " << std::endl << std::endl;
		std::cout << "Child: " << newNode << " Task: " << _task2 << std::endl;
		std::cout << "Robot: " << _agent2->GetRobot()->GetLabel() << std::endl;
		std::cout << "Time: " << _time << " Edge: " << _vid2 << " -> " << std::endl;
	}
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

	if(m_debug) {
		std::cout << "Split Edge" << std::endl;
		std::cout << "Node: " << _node << std::endl;
		std::cout << "Child: " << newNodes[0] << " Task: " << _task1 << std::endl;
		std::cout << "Robot: " << _agent1->GetRobot()->GetLabel() << std::endl;
		std::cout << "Time: " << _time << " Edge: " << _vid1 << " -> " << _vid1b << std::endl << std::endl;
		std::cout << "Child: " << newNode << " Task: " << _task2 << std::endl;
		std::cout << "Robot: " << _agent2->GetRobot()->GetLabel() << std::endl;
		std::cout << "Time: " << _time << " Edge: " << _vid2 << " -> " << _vid2b << std::endl;
	}
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
	//auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	auto tasks = this->GetTaskPlan()->GetWholeTasks();
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto plan1 = _node->GetTaskPlan(tasks[i]);
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto plan2 = _node->GetTaskPlan(tasks[j]);

			auto iter1 = plan1.begin();
			auto iter2 = plan2.begin();

			while(iter1 != plan1.end() or iter2 != plan2.end()) {
				//No agent overlap. Iterate first ending subtask
				if(iter1->m_agent != iter2->m_agent) {
					if(iter1 == plan1.end()) {
						//check if the setup path for iter2 intersects with any previous plan1 allocations
						for(auto pre = plan1.begin(); pre != iter1; pre++) {
							if(pre->m_agent != iter2->m_agent)
								continue;
							//if(iter2->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							if(iter2->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}
							/*else if(iter2->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
											and !iter2->m_setupPath.empty()
											and iter2->m_setupPath[0] != pre->m_subtaskPath.back()) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}*/
							else if(!CheckSetupPath(_node, tasks[j],
																		 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																			iter2->m_subtaskStartTime-1,
																			 pre->m_subtaskPath.back(),iter2->m_subtaskPath.front(),
																			iter2->m_agent)) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}
						}
						iter2++;
						continue;
					}
					else if(iter2 == plan2.end()) {
						//check if the setup path for iter1 intersects with any previous plan2 allocations
						for(auto pre = plan2.begin(); pre != iter2; pre++) {
							if(pre->m_agent != iter1->m_agent)
								continue;
							//if(iter1->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							if(iter1->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}
							/*else if(iter1->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
											and !iter1->m_setupPath.empty()
											and iter1->m_setupPath[0] != pre->m_subtaskPath.back()) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}*/
							else if(!CheckSetupPath(_node, tasks[i], 
																	 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																		iter1->m_subtaskStartTime-1,
																		pre->m_subtaskPath.back(),iter1->m_subtaskPath.front(),
																		iter1->m_agent)) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}
						}
						iter1++;
						continue;
					}
					else if(iter1->m_subtaskStartTime+iter1->m_subtaskPath.size() 
							== iter2->m_subtaskStartTime+iter2->m_subtaskPath.size()) {
						auto next1 = iter1;
						next1++;
			
						if(next1->m_agent == iter2->m_agent) {
							for(auto pre = plan2.begin(); pre != iter2; pre++) {
								if(pre->m_agent != iter1->m_agent)
									continue;
								//if(iter1->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								if(iter1->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
									return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
								}
								/*else if(iter1->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
												and !iter1->m_setupPath.empty()
												and iter1->m_setupPath[0] != pre->m_subtaskPath.back()) {
									return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
								}*/
								else if(!CheckSetupPath(_node, tasks[i], 
																	 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																		iter1->m_subtaskStartTime-1,
																		pre->m_subtaskPath.back(),iter1->m_subtaskPath.front(),
																		iter1->m_agent)) {
									return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
								}
							}
							iter1++;
							continue;
						}
						else {
							for(auto pre = plan1.begin(); pre != iter1; pre++) {
								if(pre->m_agent != iter2->m_agent)
									continue;
								//if(iter2->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								if(iter2->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
									return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
								}
								/*else if(iter2->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
												and !iter2->m_setupPath.empty()
												and iter2->m_setupPath[0] != pre->m_subtaskPath.back()) {
									return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
								}*/
								else if(!CheckSetupPath(_node, tasks[j], 
																		 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																			iter2->m_subtaskStartTime-1,
																			pre->m_subtaskPath.back(),iter2->m_subtaskPath.front(),
																			iter2->m_agent)) {
									return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
								}
							}
							iter2++;
							continue;
						}
				
					}
					else if(iter1->m_subtaskStartTime+iter1->m_subtaskPath.size() 
							< iter2->m_subtaskStartTime+iter2->m_subtaskPath.size()) {
						for(auto pre = plan2.begin(); pre != iter2; pre++) {
							if(pre->m_agent != iter1->m_agent)
								continue;
							//if(iter1->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							if(iter1->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}
							/*else if(iter1->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
										and !iter1->m_setupPath.empty()
										and iter1->m_setupPath[0] != pre->m_subtaskPath.back()) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}*/
							else if(!CheckSetupPath(_node, tasks[i], 
																	 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																		iter1->m_subtaskStartTime-1,
																		pre->m_subtaskPath.back(),iter1->m_subtaskPath.front(),
																		iter1->m_agent)) {
								return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
							}
						}
						iter1++;
					}
					else {
						for(auto pre = plan1.begin(); pre != iter1; pre++) {
							if(pre->m_agent != iter2->m_agent)
								continue;
							//if(iter2->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							if(iter2->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}
							/*else if(iter2->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
											and !iter2->m_setupPath.empty()
											and iter2->m_setupPath[0] != pre->m_subtaskPath.back()) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}*/
							else if(!CheckSetupPath(_node, tasks[j], 
																		 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																			iter2->m_subtaskStartTime-1,
																			pre->m_subtaskPath.back(),iter2->m_subtaskPath.front(),
																			iter2->m_agent)) {
								return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
							}
						}
						iter2++;
					}
					continue;
				}
				//Subtask 1 starts after subtask 2 is complete
				//if(iter1->m_setupStartTime >= iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()) {
				if(iter1->m_setupStartTime >= iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()) {
					//check if the setup path for iter2 intersects with any previous plan1 allocations
					for(auto pre = plan1.begin(); pre != iter1; pre++) {
						if(pre->m_agent != iter2->m_agent)
							continue;
						//if(iter2->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
						if(iter2->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
						}
						/*else if(iter2->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
								and !iter2->m_setupPath.empty()
								and iter2->m_setupPath[0] != pre->m_subtaskPath.back()) {
							return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
						}*/
						else if(!CheckSetupPath(_node, tasks[j], 
																		 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																			iter2->m_subtaskStartTime-1,
																			pre->m_subtaskPath.back(),iter2->m_subtaskPath.front(),
																			iter2->m_agent)) {
							return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*pre,*iter2, true);
						}
					}
					iter2++;
					continue;
				}
				else if(iter2->m_setupStartTime >= iter1->m_subtaskStartTime + iter1->m_subtaskPath.size()) {
					for(auto pre = plan2.begin(); pre != iter2; pre++) {
						if(pre->m_agent != iter1->m_agent)
							continue;
						//if(iter1->m_setupStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
						if(iter1->m_subtaskStartTime  < pre->m_subtaskStartTime + pre->m_subtaskPath.size()) {
							return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
						}
						/*else if(iter1->m_setupStartTime  == pre->m_subtaskStartTime + pre->m_subtaskPath.size()
										and !iter1->m_setupPath.empty()
										and iter1->m_setupPath[0] != pre->m_subtaskPath.back()) {
							return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
						}*/
						else if(!CheckSetupPath(_node, tasks[i], 
																	 pre->m_subtaskStartTime + pre->m_subtaskPath.size(),
																		iter1->m_subtaskStartTime-1,
																		pre->m_subtaskPath.back(),iter1->m_subtaskPath.front(),
																		iter1->m_agent)) {
							return CreateTaskConflictNodes(_node,tasks[j],tasks[i],*pre,*iter1, true);
						}
					}
					iter1++;
					continue;
				}
				//Subtask 2 start at the same location that subtask 1 ends
				/*else if(iter2->m_setupStartTime == iter1->m_subtaskStartTime + iter1->m_subtaskPath.size()
								and iter1->m_setupPath.front() == iter2->m_subtaskPath.back()) {
					iter1++;
					continue;
				}
				else if(iter1->m_setupStartTime == iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()
								and iter2->m_setupPath.front() == iter1->m_subtaskPath.back()) {
					iter1++;
					continue;
				}*/
				else {
					
					if(iter1->m_subtaskStartTime > iter2->m_subtaskStartTime + iter2->m_subtaskPath.size() - 1) {
						//check setup path
						if(CheckSetupPath(_node, tasks[i], 
																	 iter2->m_subtaskStartTime + iter2->m_subtaskPath.size(),
																		iter1->m_subtaskStartTime-1,
																		iter2->m_subtaskPath.back(),iter1->m_subtaskPath.front(),
																		iter1->m_agent)) {

							iter1++;
							continue;
						}
					}
					else if(iter2->m_subtaskStartTime > iter1->m_subtaskStartTime + iter1->m_subtaskPath.size() - 1) {
						//check setup path
						if(CheckSetupPath(_node, tasks[j], 
																		 iter1->m_subtaskStartTime + iter1->m_subtaskPath.size(),
																			iter2->m_subtaskStartTime-1,
																			iter1->m_subtaskPath.back(),iter2->m_subtaskPath.front(),
																			iter2->m_agent)) {


							iter2++;
							continue;
						}
					}					

					return CreateTaskConflictNodes(_node,tasks[i],tasks[j],*iter1,*iter2);
					/*auto newNode1 = new Node(_node.get(),tasks[i]);
					DiscreteAgentAllocation alloc1(iter2->m_agent, iter2->m_subtaskStartTime,
																				 iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()-1,//+1, 
																				 iter2->m_subtaskPath[0],
																				 iter2->m_subtaskPath.back());
					auto taskConflict1 = new TaskConflict(tasks[i],alloc1);
					newNode1->AddTaskConflict(tasks[i],taskConflict1);

					auto validVIDs = sg->UpdateAvailableIntervalConstraint(iter2->m_agent,
																								iter2->m_subtaskStartTime,
																				 				iter2->m_subtaskStartTime + iter2->m_subtaskPath.size()-1,//+1, 
																				 				iter2->m_subtaskPath[0],
																				 				iter2->m_subtaskPath.back(),
																								tasks[i],
																								newNode1->GetValidVIDs()[tasks[i]], 
																								newNode1->GetTaskMotionConstraints(tasks[i]));
					newNode1->UpdateValidVIDs(validVIDs.first,validVIDs.second, tasks[i]);
					newNodes.push_back(newNode1);
	
					auto newNode2 = new Node(_node.get(),tasks[j]);
					DiscreteAgentAllocation alloc2(iter1->m_agent, iter1->m_subtaskStartTime,
																				 iter1->m_subtaskStartTime + iter1->m_subtaskPath.size()-1,//+1, 
																				 iter1->m_subtaskPath[0],
																				 iter1->m_subtaskPath.back());
					auto taskConflict2 = new TaskConflict(tasks[j],alloc2);
					newNode2->AddTaskConflict(tasks[j],taskConflict2);
					validVIDs = sg->UpdateAvailableIntervalConstraint(iter1->m_agent,
																								iter1->m_subtaskStartTime,
																				 				iter1->m_subtaskStartTime + iter1->m_subtaskPath.size()-1,//+1, 
																				 				iter1->m_subtaskPath[0],
																				 				iter1->m_subtaskPath.back(),
																								tasks[j],
																								newNode2->GetValidVIDs()[tasks[j]], 
																								newNode2->GetTaskMotionConstraints(tasks[j]));
					newNode2->UpdateValidVIDs(validVIDs.first,validVIDs.second, tasks[j]);
					newNodes.push_back(newNode2);
					return newNodes;	*/
				}
				//TODO::Check if setup is the issue and can be resolved without affecting subtask start
			}
		}
	}
	return newNodes;
}


std::vector<TMPCBSNode<WholeTask,std::pair<size_t,std::pair<size_t,size_t>>>*>
TMPCBS::
CreateTaskConflictNodes(std::shared_ptr<Node> _node, WholeTask* _task1, WholeTask* _task2,
												SubtaskPlan _subtask1, SubtaskPlan _subtask2, bool _setup) {

	std::vector<Node*> newNodes;
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
					auto newNode1 = new Node(_node.get(),_task1);

					DiscreteAgentAllocation alloc1;
					if(!_setup) {
						alloc1 = DiscreteAgentAllocation(_subtask2.m_agent, _subtask2.m_subtaskStartTime,
																				 _subtask2.m_subtaskStartTime + _subtask2.m_subtaskPath.size()-1,//+1, 
																				 _subtask2.m_subtaskPath[0],
																				 _subtask2.m_subtaskPath.back());
						auto taskConflict1 = new TaskConflict(_task1,alloc1);
						newNode1->AddTaskConflict(_task1,taskConflict1);

						auto validVIDs = sg->UpdateAvailableIntervalConstraint(_subtask2.m_agent,
																								_subtask2.m_subtaskStartTime,
																				 				_subtask2.m_subtaskStartTime + _subtask2.m_subtaskPath.size()-1,//+1, 
																				 				_subtask2.m_subtaskPath[0],
																				 				_subtask2.m_subtaskPath.back(),
																								_task1,
																								newNode1->GetValidVIDs()[_task1], 
																								newNode1->GetTaskMotionConstraints(_task2));
						newNode1->UpdateValidVIDs(validVIDs.first,validVIDs.second, _task1);
					}
					else {

						if(_subtask2.m_setupPath.empty()){
							_subtask2.m_setupPath = {_subtask2.m_subtaskPath[0]};
						}

						alloc1 = DiscreteAgentAllocation(_subtask2.m_agent, _subtask2.m_setupStartTime,
																				 _subtask2.m_subtaskStartTime + _subtask2.m_subtaskPath.size()-1,//+1, 
																				 _subtask2.m_setupPath[0],
																				 _subtask2.m_subtaskPath.back());
						auto taskConflict1 = new TaskConflict(_task1,alloc1);
						newNode1->AddTaskConflict(_task1,taskConflict1);

						auto validVIDs = sg->UpdateAvailableIntervalConstraint(_subtask2.m_agent,
																								_subtask2.m_setupStartTime,
																				 				_subtask2.m_subtaskStartTime + _subtask2.m_subtaskPath.size()-1,//+1, 
																				 				_subtask2.m_setupPath[0],
																				 				_subtask2.m_subtaskPath.back(),
																								_task1,
																								newNode1->GetValidVIDs()[_task1], 
																								newNode1->GetTaskMotionConstraints(_task2));
						newNode1->UpdateValidVIDs(validVIDs.first,validVIDs.second, _task1);

					}
					newNodes.push_back(newNode1);
	
					auto newNode2 = new Node(_node.get(),_task2);
					DiscreteAgentAllocation alloc2(_subtask1.m_agent, _subtask1.m_subtaskStartTime,
																				 _subtask1.m_subtaskStartTime + _subtask1.m_subtaskPath.size()-1,//+1, 
																				 _subtask1.m_subtaskPath[0],
																				 _subtask1.m_subtaskPath.back());
					auto taskConflict2 = new TaskConflict(_task2,alloc2);
					newNode2->AddTaskConflict(_task2,taskConflict2);
					auto validVIDs = sg->UpdateAvailableIntervalConstraint(_subtask1.m_agent,
																								_subtask1.m_subtaskStartTime,
																				 				_subtask1.m_subtaskStartTime + _subtask1.m_subtaskPath.size()-1, 
																				 				_subtask1.m_subtaskPath[0],
																				 				_subtask1.m_subtaskPath.back(),
																								_task2,
																								newNode2->GetValidVIDs()[_task2], 
																								newNode2->GetTaskMotionConstraints(_task2));
					newNode2->UpdateValidVIDs(validVIDs.first,validVIDs.second, _task2);
					newNodes.push_back(newNode2);

					if(m_debug) {
						std::cout << "Parent Node: " << _node << std::endl;
						std::cout << "Child: " << newNode1 << std::endl;
						std::cout << "Task: " << _task1 << " Robot: " << alloc1.m_agent->GetRobot()->GetLabel() 
										  << " Time: " << alloc1.m_startTime << " -> " 
											<< alloc1.m_endTime << std::endl << std::endl;

						std::cout << "Child: " << newNode2 << std::endl;
						std::cout << "Task: " << _task2 << " Robot: " << alloc2.m_agent->GetRobot()->GetLabel() 
										  << " Time: " << alloc2.m_startTime << " -> " 
											<< alloc2.m_endTime << std::endl << std::endl;
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
	
	//std::unordered_map<WholeTask*,std::vector<std::pair<HandoffAgent*,size_t>>> execPaths;
/*
	for(auto task : tasks) {
		auto plan = _node->GetTaskPlan(task);
		size_t index = 0;
		execPaths[task] = {};
		for(auto subtask : plan) {
			if(subtask.m_subtaskStartTime > index) {
				for(;index < subtask.m_subtaskStartTime; index++) {
					execPaths[task].push_back(std::make_pair(nullptr,MAX_INT));
				}
			}
			for(auto vid : subtask.m_subtaskPath) {
				execPaths[task].push_back(std::make_pair(subtask.m_agent,vid));
				index++;
			}
		}
	}
*/
	std::unordered_map<WholeTask*,std::vector<std::unordered_map<HandoffAgent*,size_t>>> execPaths;

	for(auto task : tasks) {
		auto plan = _node->GetTaskPlan(task);
		execPaths[task] = {};
		std::vector<std::vector<std::pair<HandoffAgent*,size_t>>> paths;
		size_t max = 0;
		for(auto subtask : plan) {
			size_t index = 0;
			std::vector<std::pair<HandoffAgent*,size_t>> subtaskPath;
			if(subtask.m_subtaskStartTime > index) {
				for(;index < subtask.m_subtaskStartTime; index++) {
					subtaskPath.push_back(std::make_pair(nullptr,MAX_INT));
				}
			}
			for(auto vid : subtask.m_subtaskPath) {
				subtaskPath.push_back(std::make_pair(subtask.m_agent,vid));
				index++;
			}
			if(index > max)
				max = index;
			paths.push_back(subtaskPath);
		}
		size_t index = 0;
		while(index < max) {
			std::unordered_map<HandoffAgent*,size_t> step;
			bool any = false;
			for(auto path : paths) {
				if(index >= path.size())
					continue;
				if(path[index].first) {
					any = true;
					step[path[index].first] = path[index].second;
				}
			}
			if(!any) {
				step[nullptr] = MAX_INT;
			}
			execPaths[task].push_back(step);
			index++;
		}
	}





/*	
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
				agent2 = path2[index].first;

				auto vid1b = path1[index].second;
				auto vid2b = path2[index].second;

				if(!agent1 or !agent2){
					vid1 = vid1b;
					vid2 = vid2b;
					continue;
				}

			
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
*/
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
				for(;index < subtask.m_setupStartTime; index++) {
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
				if(index >= path.size())
					continue;
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
/*
	//Check setup paths against execution paths
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto execPath = execPaths[tasks[i]];
		for(size_t j = 0; j < tasks.size(); j++) {
			if(i == j)
				continue;
			
			auto setupPath = setupPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(!execPath[index].first or setupPath[index].size() == 0
					or !setupPath[index].begin()->first) {
					index++;
					continue;
				}
				for(auto agentVID : setupPath[index]) {
					auto execAgent = execPath[index].first;
					auto execVID = execPath[index].second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					auto setupAgent = agentVID.first;
					if(!setupAgent) 
						continue;
					auto setupVID = agentVID.second;	
					auto setupCfg = sg->GetCapabilityRoadmap(setupAgent)->GetVertex(setupVID);

					if(execCfg[0] == setupCfg[0] and execCfg[1] == setupCfg[1])
						return SplitVertexConflict(_node,tasks[i],tasks[j],execAgent,setupAgent,index,execVID,setupVID);

					//Check if paths continues and if so check for edge conflict
					if(index == execPath.size()-1 or index == setupPath.size()-1)
						continue;

					if(!execPath[index+1].first)
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
*/
	//Check setup paths against setup paths
	//leaving first setup path labeled exec for now because it's late and im running out of time
	//had to change the format of exec paths to match setupPaths so just check the beginning to 
	//see which comparison is being made. exec-exec, exec-setup, setup-setup

	//exec-exec	
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto execPath = execPaths[tasks[i]];
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto setupPath = execPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(execPath[index].size() == 0 or setupPath[index].size() == 0
					or !setupPath[index].begin()->first 
					or !execPath[index].begin()->first) {
					index++;
					continue;
				}
				for(auto execAgentVID : execPath[index]) {
					auto execAgent = execAgentVID.first;
					auto execVID = execAgentVID.second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					for(auto agentVID : setupPath[index]) {
						auto setupAgent = agentVID.first;
						if(!setupAgent)
							continue;

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
	//exec-setup
	for(size_t i = 0; i < tasks.size(); i++) {
		auto execPath = execPaths[tasks[i]];
		for(size_t j = 0; j < tasks.size(); j++) {
			auto setupPath = setupPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(execPath[index].size() == 0 or setupPath[index].size() == 0
					or !setupPath[index].begin()->first 
					or !execPath[index].begin()->first) {
					index++;
					continue;
				}
				for(auto execAgentVID : execPath[index]) {
					auto execAgent = execAgentVID.first;
					auto execVID = execAgentVID.second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					for(auto agentVID : setupPath[index]) {
						auto setupAgent = agentVID.first;
						if(!setupAgent)
							continue;

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

	//setup-setup
	for(size_t i = 0; i < tasks.size()-1; i++) {
		auto execPath = setupPaths[tasks[i]];
		for(size_t j = i+1; j < tasks.size(); j++) {
			auto setupPath = setupPaths[tasks[j]];

			size_t index = 0;
			
			while(index < std::min(execPath.size(),setupPath.size())) {
				if(execPath[index].size() == 0 or setupPath[index].size() == 0
					or !setupPath[index].begin()->first 
					or !execPath[index].begin()->first) {
					index++;
					continue;
				}
				for(auto execAgentVID : execPath[index]) {
					auto execAgent = execAgentVID.first;
					auto execVID = execAgentVID.second;
					auto execCfg = sg->GetCapabilityRoadmap(execAgent)->GetVertex(execVID);

					for(auto agentVID : setupPath[index]) {
						auto setupAgent = agentVID.first;
						if(!setupAgent)
							continue;

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

	PatchSetupPaths(_node.get());

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

	this->GetTaskPlan()->SetAgentAllocations({});

	auto allocs = _node->GetAgentAllocationConstraints(task);

	if(m_debug) {
		std::cout << "Allocation Constraints" << std::endl;
	}
	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	for(auto member : this->GetTaskPlan()->GetTeam()) {
		auto cfg = member->GetRobot()->GetSimulationModel()->GetState();
		int x = int(cfg[0]+.5);
		int y = int(cfg[1]+.5);
		cfg.SetData({double(x),double(y),0});
		cfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(member->GetCapability())->GetRobot());
		auto vid = sg->GetCapabilityRoadmap(member)->GetVID(cfg);
		DiscreteAgentAllocation allocation(member, 0, 0, vid, vid);
		this->GetTaskPlan()->AddAgentAllocation(member, allocation);
		for(auto alloc : allocs[member]) {
			//alloc.m_endTime = alloc.m_endTime+1;
			this->GetTaskPlan()->AddAgentAllocation(member,alloc);
			if(m_debug) {
				std::cout << member->GetRobot()->GetLabel() << " : " 
									<< alloc.m_startTime << " -> "
									<< alloc.m_endTime << std::endl;
				}
		}
	}


	auto plan = this->GetTMPTools()->GetDiscreteMAD(m_dmadLabel)->Run(task, _node->GetValidVIDs()[task],
																																		_node->GetTaskMotionConstraints(task));

	if(m_debug) {
		std::cout << "Plan for node: " << _node <<  " Task: " << task << std::endl;
		for(auto subtask : plan) {
			std::cout << subtask.m_agent->GetRobot()->GetLabel() 
								<< " Time: " << subtask.m_subtaskStartTime
								<< " -> " << subtask.m_subtaskStartTime + subtask.m_subtaskPath.size()-1
								<< std::endl;
		}
		std::cout << std::endl;
	}

	if(plan.empty())
		return false;
	_node->SetTaskPlan(task,plan);

	this->GetTaskPlan()->SetAgentAllocations({});

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

	//PatchSetupPaths(_node);
	size_t max = 0;
	for(auto taskPlan : _node->GetTaskPlans()) {
		auto end = taskPlan.second.back().m_subtaskStartTime + taskPlan.second.back().m_subtaskPath.size()+1;
		if(end > max)
			max = end;
	}

	std::unordered_map<HandoffAgent*,std::vector<Cfg>> agentPaths;
	for(auto agent : this->GetTaskPlan()->GetTeam()) {
		agentPaths[agent] = std::vector<Cfg>(max);
	}	

	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	for(auto taskPlan : _node->GetTaskPlans()) {
		for(auto subtask : taskPlan.second) {
			auto agent = subtask.m_agent;
			auto index = subtask.m_setupStartTime;
			auto roadmap = sg->GetCapabilityRoadmap(agent);

			for(auto vid : subtask.m_setupPath) {
				auto cfg = roadmap->GetVertex(vid);
				agentPaths[agent][index] = cfg;
				index++;
			}

			for(auto vid : subtask.m_subtaskPath) {
				auto cfg = roadmap->GetVertex(vid);
				agentPaths[agent][index] = cfg;
				index++;
			}
		}
	}

	for(auto& agentPath : agentPaths) {
		auto& path = agentPath.second;

		auto last = path.front();
		if(!last.GetRobot()) {
			path = {};
		}
		
		for(auto& cfg : path) {
			if(!cfg.GetRobot()) {
				cfg = last;
				continue;
			}
			last = cfg;
		}

		agentPath.first->SetPlan(agentPath.second);
	}

	for(auto agent : this->GetTaskPlan()->GetTeam()) {
		auto roadmap = sg->GetCapabilityRoadmap(agent).get();
		if(agent->GetCapability() == "land")
  		Simulation::Get()->AddRoadmap(roadmap, glutils::color::green);
		else if(agent->GetCapability() == "water")
  		Simulation::Get()->AddRoadmap(roadmap, glutils::color::blue);
	}	
	if(m_debug) {
		for(auto agentPath : agentPaths) {
			std::cout << "Path for " << agentPath.first->GetRobot()->GetLabel() << std::endl;
			for(auto cfg : agentPath.second) {
				std::cout << cfg.PrettyPrint() << std::endl;
			}
			std::cout << std::endl;
		}
	}

	auto combinedRoadmap = sg->CombinedRoadmap::GetGraph();
  Simulation::Get()->AddRoadmap(combinedRoadmap, glutils::color::brown);

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

bool
TMPCBS::
CheckSetupPath(std::shared_ptr<Node> _node, WholeTask* _task, 
							 size_t _startTime, size_t _endTime, size_t _startVID, size_t _endVID, HandoffAgent* _agent) {

	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	auto startCfg = sg->GetCapabilityRoadmap(_agent)->GetVertex(_startVID);
	auto goalCfg = sg->GetCapabilityRoadmap(_agent)->GetVertex(_endVID);

	auto path = sg->LowLevelGraphPath(startCfg, goalCfg, _node->GetTaskMotionConstraints(_task),
																		_startTime, _endTime);

	if(path.size() == 1 and _startTime - 1 == _endTime)
		return true;

	if(path.size() - 1 + _startTime > _endTime or path.empty())
		return false;
	return true;
}

void
TMPCBS::
PatchSetupPaths(Node* _node) {

	std::unordered_map<HandoffAgent*,std::list<SubtaskPlan*>> subtaskAssignments;

	std::unordered_map<SubtaskPlan*,WholeTask*> subtaskWholeTask;

	for(auto agent : this->GetTaskPlan()->GetTeam()) {
		subtaskAssignments[agent] = {};
	}
	
	for(auto& taskPlan : _node->GetTaskPlans()) {
		auto& subtasks = taskPlan.second;
		for(auto& subtask : subtasks) {
			subtaskWholeTask[&subtask] = taskPlan.first;

			auto& assignments = subtaskAssignments[subtask.m_agent];
			if(assignments.empty()) {
				assignments.push_back(&subtask);
				continue;
			}
			auto iter = assignments.begin();
			for(; iter != assignments.end(); iter++) {
				if((*iter)->m_subtaskStartTime > subtask.m_subtaskStartTime + subtask.m_subtaskPath.size() - 1) {
					assignments.insert(iter,&subtask);
					break;
				}
			}
			if(iter == assignments.end()) {
				assignments.push_back(&subtask);
			}
		}
	}

	auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

	//TODO::patch setup paths
	for(auto agentAssignments : subtaskAssignments) {
		auto assignments = agentAssignments.second;
		auto agent = agentAssignments.first;

		if(assignments.empty()) {
			continue;
		}

		auto iter = assignments.begin();

		auto startVID = this->GetTaskPlan()->GetAgentAllocations(agent)[0].m_endLocation;
		auto goalVID = (*iter)->m_subtaskPath.front();

		size_t startTime = 0;
		size_t endTime = (*iter)->m_subtaskStartTime - 1;

		auto startCfg = sg->GetCapabilityRoadmap(agent)->GetVertex(startVID);
		auto goalCfg = sg->GetCapabilityRoadmap(agent)->GetVertex(goalVID);

		auto task = subtaskWholeTask[(*iter)];

		startCfg.SetRobot(agent->GetRobot());
		goalCfg.SetRobot(agent->GetRobot());

		auto path = sg->LowLevelGraphPath(startCfg, goalCfg, _node->GetTaskMotionConstraints(task),
																		startTime, endTime);
		
		if(path.size() == 1 and startTime -1 == endTime) {
			path = {};
		}

		if(path.size() - 1 + startTime > endTime) {
			throw RunTimeException(WHERE,"Setup path invalid. Missed task conflict.");
		}

		(*iter)->m_setupPath = path;
		(*iter)->m_setupStartTime = 0;
		if(m_debug) {
			std::cout << "Rewriting setup path for agent: " << agent->GetRobot()->GetLabel() <<std::endl;
			std::cout << "Starting point: " << startVID
								<< " at time: 0"
								<< std::endl;
			
			std::cout << "Begin of subtask1: " << (*iter)->m_subtaskPath.front()
								<< " at time: " << (*iter)->m_subtaskStartTime << std::endl;

			std::cout << "Setup path: " << std::endl;
			for(auto vid : path) {
				std::cout << vid << " -> ";
			}
			std::cout << std::endl << "Length: " << path.size() << std::endl;
		}


		auto next = iter;
		next++;
		for(; next != assignments.end(); next++) {
			auto subtask1 = *iter;
			auto subtask2 = *next;

			auto startVID = subtask1->m_subtaskPath.back();
			auto goalVID = subtask2->m_subtaskPath.front();

			auto startTime = subtask1->m_subtaskStartTime + subtask1->m_subtaskPath.size();

			auto endTime = subtask2->m_subtaskStartTime - 1;			

			auto startCfg = sg->GetCapabilityRoadmap(agent)->GetVertex(startVID);
			auto goalCfg = sg->GetCapabilityRoadmap(agent)->GetVertex(goalVID);

			auto task = subtaskWholeTask[subtask2];

			auto path = sg->LowLevelGraphPath(startCfg, goalCfg, _node->GetTaskMotionConstraints(task),
																		startTime, endTime);

			if(path.size() == 1 and startTime -1 == endTime) {
				path = {};
			}

			if(path.size() - 1 + startTime > endTime) {
				throw RunTimeException(WHERE,"Setup path invalid. Missed task conflict.");
			}

			subtask2->m_setupPath = path;
			subtask2->m_setupStartTime = startTime;

			if(m_debug) {
				std::cout << "Rewriting setup path for agent: " << agent->GetRobot()->GetLabel() <<std::endl;
				std::cout << "End of subtask1: " << subtask1->m_subtaskPath.back()
									<< " at time: " << subtask1->m_subtaskStartTime + subtask1->m_subtaskPath.size() - 1 
									<< std::endl;
				
				std::cout << "Begin of subtask2: " << subtask2->m_subtaskPath.front()
									<< " at time: " << subtask2->m_subtaskStartTime << std::endl;

				std::cout << "Setup path: " << std::endl;
				for(auto vid : path) {
					std::cout << vid << " -> ";
				}
				std::cout << std::endl << "Length: " << path.size() << std::endl;
			}

			iter++;
		}
	}	

}

Decomposition* 
TMPCBS::
CreateDecomposition(std::vector<WholeTask*> _wholeTasks) {
	auto top = std::shared_ptr<SemanticTask>(new SemanticTask());

	auto decomp = new Decomposition(top);

	for(auto wholeTask : _wholeTasks) {
		auto semanticTask = std::shared_ptr<SemanticTask>(new SemanticTask(top,wholeTask->m_task));
		top->AddSubtask(semanticTask);
		decomp->AddSimpleTask(semanticTask);
		this->GetTaskPlan()->SetSemanticWholeTask(semanticTask.get(),wholeTask);
	}

	return decomp;
}
