#include "MultiTaskGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

/************************************ Construction *************************************/

MultiTaskGraph::
MultiTaskGraph(){
	this->SetName("MultiTaskGraph");
}

MultiTaskGraph::
MultiTaskGraph(XMLNode& _node) : CombinedRoadmap(_node){
	this->SetName("MultiTaskGraph");
}

/************************************ Initialization *************************************/

void
MultiTaskGraph::
Initialize(){
	auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
	m_highLevelGraph = new RoadmapGraph<Cfg,DefaultWeight<Cfg>>(robot);
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

	// Add robot-type start nodes
	for(auto& pair : _wholeTask->m_startPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == robot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		//Connect robot-type start node to the virtual start node
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
	//Add robot-type goal nodes
	for(auto& pair : _wholeTask->m_goalPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == robot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		//Connect robot-type goal node to the virtual goal node
		m_highLevelGraph->AddEdge(vid1,virtGoal,virtEdge);

		for(auto& vid2 : m_receivingVIDs[cfg.GetRobot()->GetCapability()]){
			auto w = ExtractPathWeight(vid2,vid1);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			m_highLevelGraph->AddEdge(vid2,vid1,weight);
		}
	}
	if(m_debug){
		std::cout << "Adding Task." << std::endl;
		PrintGraph();
	}	
}

void
MultiTaskGraph::
RemoveTaskFromGraph(WholeTask* _wholeTask){

	for(auto& vid : m_currentTaskVIDs){
		m_highLevelGraph->DeleteVertex(vid);
	}

	m_currentTaskVIDs = {};
	if(m_debug){
		std::cout << "Removing Task." << std::endl;
		PrintGraph();
	}	
}


/**************************************** Helpers ****************************************/
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
    
	return solution->GetPath()->Length();
}

/*------------------------------ Construction Helpers --------------------------------*/

void
MultiTaskGraph::
ConstructGraph(){
	CombinedRoadmap::ConstructGraph();
	CreateHighLevelGraph();
	if(m_debug){
		std::cout << "Initial construction." << std::endl;
		PrintGraph();
	}	
}

void 
MultiTaskGraph::
CreateHighLevelGraph(){

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();	
  m_highLevelGraph = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(robot);

  for(auto& it : this->GetTaskPlan()->GetInteractionTemplates()){
  	for(auto pair : it->GetTransformedPositionPairs()){
			auto cfgR = pair.first;//receiving
			auto cfgD = pair.second;//delivering
  		auto vidR = m_highLevelGraph->AddVertex(cfgR);
  		auto vidD = m_highLevelGraph->AddVertex(cfgD);

  		m_deliveringVIDs[cfgD.GetRobot()->GetCapability()].push_back(vidD);
  		m_receivingVIDs[cfgR.GetRobot()->GetCapability()].push_back(vidR);
  		
  		//TODO::Change this to just copy over the x,y,z position of the cfg for virtual superRobot
  		auto virtCfg = pair.first;
  		virtCfg.SetRobot(robot);
  		auto virtVID = m_highLevelGraph->AddVertex(virtCfg);
  		
  		DefaultWeight<Cfg> interactionWeight;
  		interactionWeight.SetWeight(it->GetInformation()->GetInteractionWeight());
  		m_highLevelGraph->AddEdge(vidD,virtVID,interactionWeight);
  		
  		DefaultWeight<Cfg> virtWeight;
  		virtWeight.SetWeight(-1);
  		m_highLevelGraph->AddEdge(virtVID,vidR,virtWeight);
  	}
  }

	auto dummyMap = this->GetTaskPlan()->GetDummyMap();
  for(auto dummy = dummyMap.begin(); dummy != dummyMap.end(); dummy++){
  	for(auto& vidR : m_receivingVIDs[dummy->first]){
  		for(auto& vidD : m_deliveringVIDs[dummy->first]){
  			auto w = ExtractPathWeight(vidR, vidD);
  			if(w == -1) //indicates there is no path between the two in the lower level graph
  				continue;
  			DefaultWeight<Cfg> weight;
  			weight.SetWeight(w);
  			m_highLevelGraph->AddEdge(vidR, vidD, weight);
  		}
  	}
  }

}

/********************************* Debug ******************************************/

void
MultiTaskGraph::
PrintGraph(){
	std::cout << "Printing high level vertices:" << std::endl;
	for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++){
		std::cout << vit->descriptor() << " : " << vit->property().PrettyPrint() << std::endl;
	}
	std::cout << "Printing high level edges:" << std::endl;
	for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++){
		for(auto eit = vit->begin(); eit != vit->end(); eit++){
			std::cout << eit->source() << " -> " << eit->target() << std::endl;
		}
	}
}
