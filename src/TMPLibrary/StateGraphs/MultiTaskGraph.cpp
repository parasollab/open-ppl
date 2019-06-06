#include "MultiTaskGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

/************************************ Construction *************************************/

MultiTaskGraph::
MultiTaskGraph(XMLNode& _node) : CombinedRoadmap(_node){}

/************************************ Initialization *************************************/

void
MultiTaskGraph::
Initialize(){
	CombinedRoadmap::Initialize();
}

/*************************************** Accessors ***************************************/

double
MultiTaskGraph::
ExtractPathWeight(size_t _vid1, size_t _vid2){
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	return LowLevelGraphPathWeight(start,goal);
}

double 
MultiTaskGraph::
RobotSelection(size_t _target, HandoffAgent** _minAgent){
	//Agent* minAgent = nullptr;
	double minTime = MAX_DBL;
	
	auto subtaskStart = m_highLevelGraph->GetVertex(_target);

	for(auto& agent : this->GetTaskPlan()->GetTeam()){
		//Check that robot is of the right type
		if(agent->GetRobot()->GetCapability() != subtaskStart.GetRobot()->GetCapability()){
			continue;
		}

		auto info = this->GetTaskPlan()->GetRobotAvailability(agent);
		auto location = info.first;
		auto availTime = info.second;
		
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

void
MultiTaskGraph::
AddTaskToGraph(WholeTask* _wholeTask){

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();	
	//TODO::Put a check here to make sure these are valid cfgs and already exist in the map
	Cfg start = _wholeTask->m_startPoints[robot->GetLabel()][0];
	Cfg goal = _wholeTask->m_goalPoints[robot->GetLabel()][0];

	auto virtStart = m_highLevelGraph->AddVertex(start);
	auto virtGoal = m_highLevelGraph->AddVertex(goal);

	m_currentTaskVIDs.push_back(virtStart);
	m_currentTaskVIDs.push_back(virtGoal);

	DefaultWeight<Cfg> virtEdge;
	virtEdge.SetWeight(-1);

	for(auto& pair : _wholeTask->m_startPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == robot)
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
		if(cfg.GetRobot() == robot)
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
MultiTaskGraph::
RemoveTaskFromGraph(WholeTask* _wholeTask){

	for(auto& vid : m_currentTaskVIDs){
		m_highLevelGraph->DeleteVertex(vid);
	}

	m_currentTaskVIDs = {};
}


/**************************************** Helpers ****************************************/
void 
MultiTaskGraph::
CreateHighLevelGraph(){

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();	
  m_highLevelGraph = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(robot);

  for(auto& it : this->GetTaskPlan()->GetInteractionTemplates()){
  	for(auto pair : it->GetTransformedPositionPairs()){
  		auto vid1 = m_highLevelGraph->AddVertex(pair.first);
  		auto vid2 = m_highLevelGraph->AddVertex(pair.second);

  		m_deliveringVIDs[pair.first.GetRobot()->GetCapability()];
  		m_receivingVIDs[pair.second.GetRobot()->GetCapability()];
  		
  		//TODO::Change this to just copy over the x,y,z position of the cfg for virtual superRobot
  		auto virtCfg = pair.second;
  		virtCfg.SetRobot(robot);
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

double
MultiTaskGraph::
LowLevelGraphPathWeight(Cfg _start, Cfg _goal){
	//save current library task
	auto oldTask = this->GetMPLibrary()->GetTask();
	
	auto robot = _start.GetRobot();
	std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robot));

	std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(_start.GetRobot(), _start));
    std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(_goal.GetRobot(), _goal));
    
    task->SetStartConstraint(std::move(startConstraint));
    task->AddGoalConstraint(std::move(goalConstraint));

    this->GetMPLibrary()->SetTask(task.get());

	auto dummyAgent = this->GetTaskPlan()->GetCapabilityAgent(robot->GetCapability());
	auto solution = new MPSolution(dummyAgent->GetRobot());
	solution->SetRoadmap(dummyAgent->GetRobot(),m_capabilityRoadmaps[robot->GetCapability()].get());

	this->GetMPLibrary()->Solve(this->GetMPLibrary()->GetMPProblem(), task.get(), solution, "EvaluateMapStrategy",
                       LRand(), "LowerLevelGraphWeight");
    
    if(this->GetMPLibrary()->GetPath()->Cfgs().empty())
    	return -1;

	//restore library task
    this->GetMPLibrary()->SetTask(oldTask);
    
	return this->GetMPLibrary()->GetPath()->Length();
}

