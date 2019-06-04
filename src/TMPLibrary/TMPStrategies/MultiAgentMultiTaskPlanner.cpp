#include "MultiAgentMultiTaskPlanner.h"

#include "TMPLibrary/TaskPlan.h"

/*****************************************Constructor****************************************************/
MultiAgentMultiTaskPlanner::
MultiAgentMultiTaskPlanner(XMLNode& _node) : TMPStrategyMethod(_node){ }


/******************************************Configure*****************************************************/



/*****************************************Call Method****************************************************/

TaskPlan*
MultiAgentMultiTaskPlanner::
PlanTasks(MPLibrary* _library, vector<HandoffAgent*> _agents,
                               vector<std::shared_ptr<MPTask>> _tasks){
	TMPStrategyMethod::PlanTasks(_library,_agents,_tasks);
	
	m_library->GetTask()->SetRobot(m_robot);

	CreateHighLevelGraph();
	
	InitializeRAT();

	for(auto& wholeTask : this->GetTaskPlan()->GetWholeTasks()){
		AddTaskToGraph(wholeTask);
		TaskPlan* taskPlan = new TaskPlan();

		//Find Task Plan

		m_taskPlans.push_back(taskPlan);
		RemoveTaskFromGraph(wholeTask);
	}
	return new TaskPlan();
}

/*****************************************TaskGraph Functions****************************************************/

void 
MultiAgentMultiTaskPlanner::
CreateHighLevelGraph(){

  m_highLevelGraph = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);

  for(auto& it : m_solution->GetInteractionTemplates()){
  	for(auto pair : it->GetTransformedPositionPairs()){
  		auto vid1 = m_highLevelGraph->AddVertex(pair.first);
  		auto vid2 = m_highLevelGraph->AddVertex(pair.second);

  		m_deliveringVIDs[pair.first.GetRobot()->GetCapability()];
  		m_receivingVIDs[pair.second.GetRobot()->GetCapability()];
  		
  		//TODO::Change this to just copy over the x,y,z position of the cfg for virtual superRobot
  		auto virtCfg = pair.second;
  		virtCfg.SetRobot(m_robot);
  		auto virtVID = m_highLevelGraph->AddVertex(virtCfg);
  		
  		DefaultWeight<Cfg> interactionWeight;
  		interactionWeight.SetWeight(it->GetInformation()->GetInteractionWeight());
  		m_highLevelGraph->AddEdge(vid1,virtVID,interactionWeight);
  		
  		DefaultWeight<Cfg> virtWeight;
  		virtWeight.SetWeight(-1);
  		m_highLevelGraph->AddEdge(virtVID,vid2,virtWeight);
  	}
  }

	auto dummyMap = this->GetTaskPlan()->GetDummyMap();
  for(auto dummy = dummyMap.begin(); dummy != dummyMap.end(); dummy++){
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
MAMTDijkstra(WholeTask* _wholeTask){
	SSSPPathWeightFunction<TaskGraph> weight;
	weight = [this](typename TaskGraph::adj_edge_iterator& _ei,
                   const double _sourceDistance,
                   const double _targetDistance) {
            return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance);
        };

	//TODO::Actually call dijkstras
	
	
	return TaskPlan();
}

void
MultiAgentMultiTaskPlanner::
AddTaskToGraph(WholeTask* _wholeTask){

	//TODO::Put a check here to make sure these are valid cfgs and already exist in the map
	Cfg start = _wholeTask->m_startPoints[m_robot->GetLabel()][0];
	Cfg goal = _wholeTask->m_goalPoints[m_robot->GetLabel()][0];

	auto virtStart = m_highLevelGraph->AddVertex(start);
	auto virtGoal = m_highLevelGraph->AddVertex(goal);

	m_currentTaskVIDs.push_back(virtStart);
	m_currentTaskVIDs.push_back(virtGoal);

	DefaultWeight<Cfg> virtEdge;
	virtEdge.SetWeight(-1);

	for(auto& pair : _wholeTask->m_startPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == m_robot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		m_highLevelGraph->AddEdge(virtStart,vid1,virtEdge);

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
	for(auto& pair : _wholeTask->m_goalPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == m_robot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		m_highLevelGraph->AddEdge(vid1,virtGoal,virtEdge);

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
RemoveTaskFromGraph(WholeTask* _wholeTask){

	for(auto& vid : m_currentTaskVIDs){
		m_highLevelGraph->DeleteVertex(vid);
	}

	m_currentTaskVIDs = {};
}

/*****************************************RAT Functions****************************************************/

void
MultiAgentMultiTaskPlanner::
InitializeRAT(){
	auto dummyMap = this->GetTaskPlan()->GetDummyMap();
	for(auto dummy = dummyMap.begin(); dummy != dummyMap.end(); dummy++){
		m_RAT[dummy->second] = std::pair<Cfg,double>(
										dummy->second->GetRobot()->GetSimulationModel()->GetState(),
										0.0);
	}
}

/*****************************************Helper Functions****************************************************/

double
MultiAgentMultiTaskPlanner::
ExtractPathWeight(size_t _vid1, size_t _vid2){
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	return LowLevelGraphPathWeight(start,goal);
}

double
MultiAgentMultiTaskPlanner::
LowLevelGraphPathWeight(Cfg _start, Cfg _goal){
	//save current library task
	auto oldTask = m_library->GetTask();
	
	auto robot = _start.GetRobot();
	std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robot));

	std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(_start.GetRobot(), _start));
    std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(_goal.GetRobot(), _goal));
    
    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));

    m_library->SetTask(task.get());

	auto dummyAgent = this->GetTaskPlan()->GetCapabilityAgent(robot->GetCapability());
	auto solution = new MPSolution(dummyAgent->GetRobot());
	solution->SetRoadmap(dummyAgent->GetRobot(),m_capabilityRoadmaps[robot->GetCapability()].get());

	m_library->Solve(m_library->GetMPProblem(), task.get(), solution, "EvaluateMapStrategy",
                       LRand(), "LowerLevelGraphWeight");
    
    if(m_library->GetPath()->Cfgs().empty())
    	return -1;

	//restore library task
    m_library->SetTask(oldTask);
    
	return m_library->GetPath()->Length();
}

double
MultiAgentMultiTaskPlanner::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
	const double _sourceDistance, const double _targetDistance) {
  
  const double edgeWeight  = _ei->property().GetWeight();
  double readyTime = 0;
  Agent* newAgent = nullptr;
  size_t source = _ei->source();
  size_t target = _ei->target();
  
  //Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  bool virt = (edgeWeight == -1);
  if(virt){
  	readyTime = RobotSelection(target,&newAgent);
  }
  else{
  	newAgent = m_nodeAgentMap[source];
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
  	m_nodeAgentMap[target] = newAgent;
  }
  return newDistance;
}

double 
MultiAgentMultiTaskPlanner::
RobotSelection(size_t _target, Agent** _minAgent){
	//Agent* minAgent = nullptr;
	double minTime = MAX_DBL;
	
	auto subtaskStart = m_highLevelGraph->GetVertex(_target);

	for(auto agentInfo : m_RAT){
		auto agent = agentInfo.first;
		//Check that robot is of the right type
		if(agent->GetRobot()->GetCapability() != subtaskStart.GetRobot()->GetCapability()){
			continue;
		}

		auto location = agentInfo.second.first;
		auto availTime = agentInfo.second.second;
		
		auto travelTime = LowLevelGraphPathWeight(location,subtaskStart);

		auto readyTime = availTime + travelTime;

		if(readyTime < minTime){
			*_minAgent = agent;
			minTime = readyTime;
		}
	}
	//m_nodeAgentMap[_ei->target()] = minAgent;
	return minTime;
}











