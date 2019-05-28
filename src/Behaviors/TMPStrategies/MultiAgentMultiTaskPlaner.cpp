#include "MultiAgentMultiTaskPlanner.h"


/*****************************************Constructor****************************************************/
MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(){
  m_useITs = _node.Read("useITs", false, m_useITs,
                        "Indicate if the TMP Strategy should use ITs when planning.");
  m_debug = _node.Read("debug", false, m_debug,
                       "Indicate if the TMP Strategy should output debug information.");

};

/*****************************************Call Method****************************************************/

TaskPlann
MultiAgentMultiTaskPlanner::
PlanTask(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks, 
                               Robot* _superRobot,
                               std::unordered_map<std::string, 
                               std::unique_ptr<PlacementMethod>>* _ITPlacementMethods = nullptr){
	
	TMPStrategyMethod::Initialize();

	m_library->SetRobot(_superRobot);

	CreateHighLevelGraph(_superRobot);
	
	InitializeRAT();

	for(auto& wholeTask : m_wholeTasks){
		AddTaskToGraph(wholeTask);
		TaskPlan* taskPlan = new TaskPlan();

		//Find Task Plan

		m_taskPlans.push_back(taskPlan);
		RemoveTaskFromGraph(wholeTask);
	}

}

/*****************************************TaskGraph Functions****************************************************/

void 
MultiAgentMultiTaskPlanner::
CreateHighLevelGraph(Robot* _superRobot){

  m_highLevelGraph = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(_superRobot);

  for(auto& it : m_solution->GetInteractionTemplates()){
  	for(auto pair : it->GetTransformedPositionPairs()){
  		vid1 = m_highLevelGraph.AddVertex(pair.first);
  		vid2 = m_highLevelGraph.AddVertex(pair.second);

  		m_deliveringVIDs[pair.first->GetRobot()->GetCapability()];
  		m_receivingVIDs[pair.second->GetRobot()->GetCapability()];
  		
  		//TODO::Change this to just copy over the x,y,z position of the cfg for virtual superRobot
  		virtCfg = pair.second;
  		virtCfg.SetRobot(_superRobot);
  		virt = m_highLevelGraph.AddVertex(virtCfg);
  		
  		DefaultWeight<Cfg> interactionWeight;
  		interactionWeight.SetWeight(it->GetInformation()->GetInteractionWeight());
  		m_highLevelGraph.AddEdge(vid1,virtCfg,interactionWeight);
  		
  		DefaultWeight<Cfg> virtWeight;
  		virtWeight.SetWeight(-1);
  		m_highLevelGraph.AddEdge(virtCfg,vid2,virtWeight);
  	}
  }

  for(auto& dummy = m_dummyMap->begin(); dummy != m_dummyMap->end(); dummy++){
  	for(auto& vid1 : m_receivingVIDs[dummy->first]){
  		for(auto& vid2 : m_deliveringVIDs[dummy->first]){
  			auto w = ExtractPathWeight(vid1, vid2);
  			if(w == -1) //indicates there is no path betweenthe two in the lower level graph
  				continue;
  			DefaultWeight<Cfg> weight;
  			weight.SetWeight(w);
  			m_highLevelGraph->AddEdge(vid1, vid2, weight);
  		}
  	}
  }

}

TaskPlan
MultiAgentMultiTaskPlanner::
MAMTDijkstra(WholeTask& _wholeTask){
	SSSPPathWeightFunction<TaskGraph> weight;
	weight = [this](typename TaskGraph::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->MAMTPathWeight(_ei);
        };
}

void
MultiAgentMultiTaskPlanner::
AddTaskToGraph(WholeTask& _wholeTask, Robot* _superRobot){

	//TODO::Put a check here to make sure these are valid cfgs and already exist in the map
	Cfg start = wholeTask.m_startPoints[_suoerRobot->GetLabel()];
	Cfg goal = wholeTask.m_goalPoints[_suoerRobot->GetLabel()];

	auto virtStart = m_highLevelGraph->AddVertex(start);
	auto virtGoal = m_highLevelGraph->AddVertex(goal);

	m_currentTaskVIDs.push_back(virtStart);
	m_currentTaskVIDs.push_back(virtGoal);

	DefaultWeight<Cfg> virtEdge;
	virtEdge.SetWeight(-1);

	for(auto& cfg : _wholeTask.m_startPoints){
		if(cfg.GetRobot() == _superRobot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		m_highLevelGraph->AddEdge(virtStart,vid1,virtEdge)

		for(auto& vid2 : m_deliveringVIDs[cfg.GetRobot()->GetCapability()]){
			auto w = ExtractPathWeight(vid1,vid2);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			m_highLevelGraph->AddEdge(vid1,vid2,weight);
		}
	}

	//TODO::Double check that this is implemented corectly (copied and pasted then changed from above block)
	for(auto& cfg : _wholeTask.m_goalPoints){
		if(cfg.GetRobot() == _superRobot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		m_highLevelGraph->AddEdge(vid1,virtGoal,virtEdge)

		for(auto& vid2 : m_deliveringVIDs[cfg.GetRobot()->GetCapability()]){
			auto w = ExtractPathWeight(vid2,vid1);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			m_highLevelGraph->AddEdge(vid2,vid1,weight);
		}
	}
}

void
MultiAgentMultiTaskPlanner::
RemoveTaskFromGraph(WholeTask& _wholeTask){

	for(auto& vid : m_currentTaskVIDs){
		m_highLevelGraph->DeleteVertex(vid);
	}

	m_currentTaskVIDs = {};
}

/*****************************************RAT Functions****************************************************/

void
MultiAgentMultiTaskPlanner::
InitializeRAT(){
	for(auto dummy = m_dummyMap.begin(); dummy != m_dummyMap.end(); dummy++){
		m_RAT[dummy->second] = std::pair<Cfg,Double>(dummy->first,0.0);
	}
}

/*****************************************Helper Functions****************************************************/

Double
MultiAgentMultiTaskPlanner::
ExtractPathWeight(size_t _vid1, size_t _vid2){
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	return LowLevelGraphPathWeight(start,goal);
}

Double
MultiAgentMultiTaskPlanner::
LowLevelGraphPathWeight(Cfg _start, Cfg _goal){
	auto robot = m_library->GetRobot();
	std::shared_ptr<MPTask> task(robot);

	std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(_start->GetRobot(), _start));
    std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(_goal->GetRobot(), _goal));
    
    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));

    m_library->SetTask(task);

	m_library->Solve(m_library->GetMPProblem(), task, solution, "EvaluateMapStrategy",
                       LRand(), "LowerLevelGraphWeight");
    
    if(m_library->GetPath()->Cfgs().empty())
    	return -1;

    return m_library->GetPath()->Length();
}

double
MultiAgentMultiTaskPlanner::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
	const double _sourceDistance, const double _targetDistance) const {
  //TODO::Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  
  const double edgeWeight  = _ei->property().GetWeight();
  bool virt = (edgeWeight == -1);
  double readyTime = 0;
  Agent* newAgent = nullptr;
  
  if(virt){
  	readyTime = RobotSelection(_ei,newAgent);
  }
  else{
  	newAgent = m_nodeAgentMap[_ei->source()];
  }

  //TODO::Then do the rest of the regular dijkstra stuff
  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  double newDistance;
  if(virt){
  	//Check if the robot will need extra time to reach the location
  	if(readyTime > _sourceDistance){
  		newDistance = readyTime;
  	}
  	//Or if it can be there when the previous robot reaches the interaction point
  	else{
  		newDistance = _sourceDistance;
  	}
  }
  else{
  	newDistance = _sourceDistance + edgeWeight;
  }

  // If this edge is better than the previous we update the robot at the node
  if(newDistance < _targetDistance) {
  	m_nodeAgentMap[_ei->target()] = newAgent;
  }
  return newDistance;
}

double 
MultiAgentMultiTaskPlanner::
RobotSelection(typename TaskGraph::adj_edge_iterator& _ei, (Agent*)& _minAgent){
	//Agent* minAgent = nullptr;
	double minTime = MAX_DBL;
	
	auto subtaskStart = _ei->target()->property();

	for(auto agentInfo : m_RAT){
		auto agent = agentInfo.first;
		//Check that robot is of the right type
		if(agent->GetRobot()->GetCapability() != subtaskStart->GetRobot()->GetCapability()){
			continue;
		}

		auto location = agentInfo.second.first
		auto availTime = agentInfo.second.second
		
		auto travelTime = LowLevelGraphPathWeight(location,subtaskStart)

		auto readyTime = availTime + travelTime;

		if(readyTime < minTime){
			_minAgent = agent;
			minTime = readyTime;
		}
	}
	//m_nodeAgentMap[_ei->target()] = minAgent;
	return minTime;
}











