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

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
MultiTaskGraph::
GetGraph(){
	return m_highLevelGraph;
}

double
MultiTaskGraph::
ExtractPathWeight(size_t _vid1, size_t _vid2){
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	return LowLevelGraphPathWeight(start,goal);
}

double 
MultiTaskGraph::
RobotSelection(size_t _source, size_t _target, Agent** _minAgent,
							std::unordered_map<Agent*,std::list<OccupiedInterval*>>& _RATCache,
							size_t _parent, double _previousEdge){
	//Agent* minAgent = nullptr;
	double minTime = MAX_DBL;
/*	
	auto subtaskStart = m_highLevelGraph->GetVertex(_target);

	for(auto& agent : this->GetTaskPlan()->GetTeam()){
		//Check that robot is of the right type
		if(agent->GetRobot()->GetCapability() != subtaskStart.GetRobot()->GetCapability()){
			continue;
		}

		//std::pair<Cfg,double> info;
		//auto cache = _RATCache.find(agent); 
		//(cache != _RATCache.end()) ? info = cache->second 
		//													 : info = this->GetTaskPlan()->GetRobotAvailability(agent);
		
		auto location = info.first;
		auto availTime = info.second;
		
		auto travelTime = LowLevelGraphPathWeight(location,subtaskStart);
		if(travelTime == -1)
			continue;

		auto readyTime = availTime + travelTime;

		if(readyTime < minTime){
			*_minAgent = agent;
			minTime = readyTime;
		}
	}
	//m_nodeAgentMap[_ei->target()] = minAgent;
	*/

	// TODO::Will need to add checks if intuition that parent and previous edge will always 
	// exist turns out to be false
	std::cout << "Finding next robot" << std::endl;	
	auto subtaskStart = m_highLevelGraph->GetVertex(_source);
	for(auto& agent : this->GetTaskPlan()->GetTeam()){
		//Check that robot is of the right type
		if(agent->GetRobot()->GetCapability() != subtaskStart.GetRobot()->GetCapability()){
			continue;
		}
	
		std::list<OccupiedInterval*> cache = _RATCache[agent];
		auto intervals = this->GetTaskPlan()->GetRobotAvailability(agent);
		intervals.merge(cache);
		
		for(auto it = intervals.begin(); it != intervals.end(); it++) {
			auto interval = *it;
			double availableTime = interval->GetEndTime();

			Cfg destination = m_highLevelGraph->GetVertex(_source);
			std::cout << "Robot type of destination: " << destination.GetRobot()->GetCapability() << std::endl; 
			double setupTime = LowLevelGraphPathWeight(interval->GetEndLocation(),destination);
			std::cout << "Setup time: " << setupTime << std::endl;
			double returnTime = 0;
			double nextIntervalStart = MAX_DBL;
			auto next = it;
			next++;
			if(next != intervals.end()) {
				nextIntervalStart = (*next)->GetStartTime();
				
				auto returnLocation = m_highLevelGraph->GetVID((*next)->GetStartLocation());
				returnTime = m_highLevelGraph->GetEdge(_parent,returnLocation).GetWeight();
			}
		
			double timeToComplete = availableTime + setupTime + _previousEdge + returnTime;
			//check if this robot can complete the subtask before starting it's next interval
			if(timeToComplete > nextIntervalStart)
				continue;			
			auto readyTime = availableTime+setupTime;
			if(readyTime < minTime){
				minTime = readyTime;
				*_minAgent = agent;
			}
			break;
		}
		
	}

	if(!*_minAgent)
		throw RunTimeException(WHERE, "No viable agent was found in Robot Selection.");
	return minTime;
}

std::pair<size_t,size_t>
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

	std::unordered_map<std::string,size_t> startVIDs;
	// Add robot-type start nodes
	for(auto& pair : _wholeTask->m_startPoints){
		auto cfg = pair.second[0];
		if(cfg.GetRobot() == robot)
			continue;
		auto vid1 = m_highLevelGraph->AddVertex(cfg);
		m_currentTaskVIDs.push_back(vid1);
		startVIDs[cfg.GetRobot()->GetCapability()] = vid1;
		//Connect robot-type start node to the virtual start node
		//m_highLevelGraph->AddEdge(virtStart,vid1,virtEdge);
		m_highLevelGraph->AddEdge(vid1,virtStart,virtEdge);

		for(auto& vid2 : m_deliveringVIDs[cfg.GetRobot()->GetCapability()]){
			auto w = ExtractPathWeight(vid1,vid2);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			//m_highLevelGraph->AddEdge(vid1,vid2,weight);
			m_highLevelGraph->AddEdge(vid2,vid1,weight);
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
		//m_highLevelGraph->AddEdge(vid1,virtGoal,virtEdge);
		m_highLevelGraph->AddEdge(virtGoal,vid1,virtEdge);

		for(auto& vid2 : m_receivingVIDs[cfg.GetRobot()->GetCapability()]){
			auto w = ExtractPathWeight(vid2,vid1);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			//m_highLevelGraph->AddEdge(vid2,vid1,weight);
			m_highLevelGraph->AddEdge(vid1,vid2,weight);
		}
		//Attempt to directly connect the goal to the start
		auto vit = startVIDs.find(cfg.GetRobot()->GetCapability());
		if(vit != startVIDs.end()){
			auto w = ExtractPathWeight(vit->second,vid1);
			if(w == -1)
				continue;
			DefaultWeight<Cfg> weight;
			weight.SetWeight(w);
			//m_highLevelGraph->AddEdge(vit->second,vid1,weight);
			m_highLevelGraph->AddEdge(vid1,vit->second,weight);
		}
	}
	if(m_debug){
		std::cout << "Adding Task." << std::endl;
		PrintGraph();
	}
	return std::pair<size_t,size_t>(virtStart,virtGoal);	
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

	if(_start.GetRobot()->GetCapability() != _goal.GetRobot()->GetCapability()){
		throw RunTimeException(WHERE, "start and goal are of mismatched robot types.");
	}

	//save current library task
	auto oldTask = this->GetMPLibrary()->GetTask();
	
	auto dummyAgent = this->GetTaskPlan()->GetCapabilityAgent(_start.GetRobot()->GetCapability());
	auto robot = dummyAgent->GetRobot();

	_start.SetRobot(robot);
	_goal.SetRobot(robot);

	std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robot));

	std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(robot, _start));
	std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(robot, _goal));

	task->SetStartConstraint(std::move(startConstraint));
	task->AddGoalConstraint(std::move(goalConstraint));

	this->GetMPLibrary()->SetTask(task.get());

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
  		auto virtCfg = pair.second;
  		virtCfg.SetRobot(robot);
  		auto virtVID = m_highLevelGraph->AddVertex(virtCfg);
  		
  		DefaultWeight<Cfg> interactionWeight;
  		interactionWeight.SetWeight(it->GetInformation()->GetInteractionWeight());
  		//m_highLevelGraph->AddEdge(vidD,virtVID,interactionWeight);
  		m_highLevelGraph->AddEdge(virtVID,vidD,interactionWeight);
  		
  		DefaultWeight<Cfg> virtWeight;
  		virtWeight.SetWeight(-1);
  		//m_highLevelGraph->AddEdge(virtVID,vidR,virtWeight);
  		m_highLevelGraph->AddEdge(vidR,virtVID,virtWeight);
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
  			//m_highLevelGraph->AddEdge(vidR, vidD, weight);
  			m_highLevelGraph->AddEdge(vidD, vidR, weight);
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
		std::cout << vit->descriptor() << " : " 
							<< vit->property().PrettyPrint() << " : " 
							<< vit->property().GetRobot()->GetLabel() << std::endl;
	}
	std::cout << "Printing high level edges:" << std::endl;
	for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++){
		for(auto eit = vit->begin(); eit != vit->end(); eit++){
			std::cout << eit->source() << " -> " 
								<< eit->target() << " : " 
								<< eit->property().GetWeight() << std::endl;
		}
	}
	std::cout << "Robot Availability Table:" << std::endl;
	for(auto& ra : this->GetTaskPlan()->GetRAT()){
		std::cout << ra.first << std::endl;
		for(auto inter : ra.second) {
			std::cout << inter->Print() << std::endl;
		}
	}
}
