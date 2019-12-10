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
  m_highLevelGraph = std::shared_ptr<TaskGraph>(new TaskGraph(robot));
  CombinedRoadmap::Initialize();
}

/*************************************** Accessors ***************************************/

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
MultiTaskGraph::
GetGraph(){
  return m_highLevelGraph.get();
}

std::shared_ptr<RoadmapGraph<Cfg,DefaultWeight<Cfg>>>
MultiTaskGraph::
GetAvailableIntervalGraph() {
  return m_availableIntervalGraph;
}

std::pair<Agent*,std::pair<double,double>>
MultiTaskGraph::
GetInterval(size_t _vid){
  return m_agentAvailableIntervalMap.at(_vid);
}

double
MultiTaskGraph::
ValidTransition(size_t _source, size_t _target, double _edge,
                double _sourceDistance, std::pair<double,Cfg> _update) {
  //if(_update.first == 0)
  //  return _edge;
  auto sourceInterval = m_agentAvailableIntervalMap.at(_source);

  if(_sourceDistance > sourceInterval.second.second+.000001 or
     _sourceDistance < sourceInterval.second.first-.000001)
    return -1;

  auto targetInterval = m_agentAvailableIntervalMap.at(_target);

  if(_sourceDistance + _edge > targetInterval.second.second+.000001)// or
     //_sourceDistance + _edge < targetInterval.second.first)//dont use bc you can wait
    return -1;

  auto sourceCfg = m_availableIntervalGraph->GetVertex(_source);
  auto targetCfg = m_availableIntervalGraph->GetVertex(_target);
  //Subtask start and end
  if(sourceCfg.GetRobot() == targetCfg.GetRobot())
    return _edge;
  //Interaction and transition between subtasks - may have waiting time
  double availTime;
  if(_update.first == 0)
    availTime = targetInterval.second.first;
  else
    availTime = _update.first
              + LowLevelGraphPathWeight(_update.second,targetCfg);

  //Check if the extra waiting time violates the delivering or receiving
  //interval
  if(availTime > targetInterval.second.second or
     availTime > sourceInterval.second.second)
    return -1;

  //TODO::I think this is implicitly done by failing the next check anyways
  //Check if there will be no waiting time
  //if(availTime < targetInterval.second.first)
  //  return _edge;//should be 0 unless bias is changed

  //Check if the delivering robot will need to wait
  if(availTime > _sourceDistance + _edge)
    return availTime - _sourceDistance;

  return _edge;
}

double
MultiTaskGraph::
ExtractPathWeight(size_t _vid1, size_t _vid2, bool _forceMatch){
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	if(_forceMatch){
		if(start.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			start.SetRobot(goal.GetRobot());
		else if (goal.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			goal.SetRobot(start.GetRobot());
	}

	return LowLevelGraphPathWeight(start,goal);
}

double
MultiTaskGraph::
RobotSelection(size_t _source, size_t _target, Agent** _minAgent,
    std::unordered_map<Agent*,std::list<OccupiedInterval>>& _RATCache,
    size_t _parent, double _previousEdge, std::set<HandoffAgent*> _usedAgents){
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
  /*
  std::cout << "Finding next robot" << std::endl;
  auto subtaskStart = m_highLevelGraph->GetVertex(_source);
  for(auto& agent : this->GetTaskPlan()->GetTeam()){
    //Check that robot is of the right type
    if(agent->GetRobot()->GetCapability() != subtaskStart.GetRobot()->GetCapability()){
      continue;
    }

    if(_usedAgents.count(agent)) //TODO:: fix - part of hack to not reuse agents
      continue;

    std::list<OccupiedInterval*> cache = _RATCache[agent];
    auto intervals = this->GetTaskPlan()->GetRobotAvailability(agent);
    //intervals.merge(cache);

    for(auto it = intervals.begin(); it != intervals.end(); it++) {
      auto interval = *it;
      double availableTime = interval.GetEndTime();

      Cfg destination = m_highLevelGraph->GetVertex(_source);
      std::cout << "Robot type of destination: " << destination.GetRobot()->GetCapability() << std::endl;
      double setupTime = LowLevelGraphPathWeight(interval.GetEndLocation(),destination);
      std::cout << "Setup time: " << setupTime << std::endl;
      double returnTime = 0;
      double nextIntervalStart = MAX_DBL;
      auto next = it;
      next++;
      if(next != intervals.end()) {
        nextIntervalStart = (*next).GetStartTime();

        auto parentLocation = m_highLevelGraph->GetVertex(_parent);
        returnTime = LowLevelGraphPathWeight(parentLocation,(*next).GetStartLocation());
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

  }*/

	/*auto targetCfg = m_highLevelGraph->GetVertex(_target);
	for(auto agentAvailableIntervals : GetAgentAvailableIntervals(_source,_target,_RATCache)){
    if(agentAvailableIntervals.first->GetRobot()->GetCapability() != targetCfg.GetRobot()->GetCapability())
			continue;

		for(auto& interval : agentAvailableIntervals.second){
			auto startTime = interval.first;
			auto travelTime = _previousEdge;

			// Get the cost of going to start of next assignment from end this possible one
			double returnTime = 0;
			if(interval.second.GetStartTime() != MAX_DBL)
				returnTime = LowLevelGraphPathWeight(targetCfg,interval.second.GetStartLocation());

			auto totalTime = startTime+travelTime+returnTime;
			if(totalTime > interval.second.GetStartTime())
				continue;

			if(startTime < minTime){
				minTime = startTime;
				*_minAgent = agentAvailableIntervals.first;
			}
			break;
		}
	}*/

  if(!*_minAgent)
    throw RunTimeException(WHERE, "No viable agent was found in Robot Selection.");
  return minTime;
}

std::pair<size_t,size_t>
MultiTaskGraph::
AddTaskToGraph(WholeTask* _wholeTask,std::set<size_t> _validAigVIDs){

  /*if(m_currentTask) {
    for(auto vid : m_hlgTaskVIDs[m_currentTask]) {
      m_highLevelGraph->SetVertexInvalidated(vid);
    }
    for(auto vid : m_aigTaskVIDs[m_currentTask]) {
      m_availableIntervalGraph->SetVertexInvalidated(vid);
    }
 	} */
  m_currentTask = _wholeTask;
  if(!m_hlgTaskVIDs[_wholeTask].empty()) {
    /*for(auto vid : m_hlgTaskVIDs[_wholeTask]) {
      m_highLevelGraph->SetVertexInvalidated(vid,false);
    }
    for(auto vid : m_aigTaskVIDs[_wholeTask]) {
      if(_validAigVIDs.empty() or _validAigVIDs.count(vid))
        m_availableIntervalGraph->SetVertexInvalidated(vid,false);
    }
		if(!_validAigVIDs.empty())
			for(auto vit = m_availableIntervalGraph->begin(); vit != m_availableIntervalGraph->end(); vit++) {
				if(_validAigVIDs.count(vit->descriptor()))
					m_availableIntervalGraph->SetVertexInvalidated(vit->descriptor(),false);
				else
					m_availableIntervalGraph->SetVertexInvalidated(vit->descriptor(),true);
			}
  	if(m_debug){
    	std::cout << "Adding Task." << std::endl;
    	PrintGraph();
    	//PrintAvailabilityGraph();
  	}*/
  	return m_taskQuery[_wholeTask];
    //return std::pair<size_t,size_t>(m_hlgTaskVIDs[_wholeTask][0],
    //                                m_hlgTaskVIDs[_wholeTask][1]);
  }

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  //TODO::Put a check here to make sure these are valid cfgs and already exist in the map
  Cfg start = _wholeTask->m_startPoints[robot->GetLabel()][0];
  Cfg goal = _wholeTask->m_goalPoints[robot->GetLabel()][0];

  auto virtStart = m_highLevelGraph->AddDuplicateVertex(start);
  auto virtGoal = m_highLevelGraph->AddDuplicateVertex(goal);

	m_taskQuery[_wholeTask] = std::make_pair(virtStart,virtGoal);

  m_currentTaskVIDs.push_back(virtStart);
  m_currentTaskVIDs.push_back(virtGoal);

  DefaultWeight<Cfg> startEdge;
  startEdge.SetWeight(0);

  std::unordered_map<std::string,size_t> startVIDs;
  // Add robot-type start nodes
  for(auto& pair : _wholeTask->m_startPoints){
    auto cfg = pair.second[0];
    if(cfg.GetRobot() == robot)
      continue;
    auto vid1 = m_highLevelGraph->AddDuplicateVertex(cfg);
    m_currentTaskVIDs.push_back(vid1);
    startVIDs[cfg.GetRobot()->GetCapability()] = vid1;
    //Connect robot-type start node to the virtual start node
    m_highLevelGraph->AddEdge(virtStart,vid1,startEdge);

    for(auto& vid2 : m_deliveringVIDs[cfg.GetRobot()->GetCapability()]){
      auto w = ExtractPathWeight(vid1,vid2);
      if(w == -1)
        continue;
      DefaultWeight<Cfg> weight;
      weight.SetWeight(w);
      m_highLevelGraph->AddEdge(vid1,vid2,weight);
    }
  }
  //Add robot-type goal nodes
  for(auto& pair : _wholeTask->m_goalPoints){
    auto cfg = pair.second[0];
    if(cfg.GetRobot() == robot)
      continue;
    auto vid1 = m_highLevelGraph->AddDuplicateVertex(cfg);
    m_currentTaskVIDs.push_back(vid1);
    m_goalDelivering.insert(vid1);
    //Connect robot-type goal node to the virtual goal node
    m_highLevelGraph->AddEdge(vid1,virtGoal,startEdge);
    //m_highLevelGraph->AddEdge(virtGoal,vid1,virtEdge);

    for(auto& vid2 : m_receivingVIDs[cfg.GetRobot()->GetCapability()]){
      auto w = ExtractPathWeight(vid2,vid1);
      if(w == -1)
        continue;
      DefaultWeight<Cfg> weight;
      weight.SetWeight(w);
      m_highLevelGraph->AddEdge(vid2,vid1,weight);
    }
    //Attempt to directly connect the goal to the start
    auto vit = startVIDs.find(cfg.GetRobot()->GetCapability());
    if(vit != startVIDs.end()){
      auto w = ExtractPathWeight(vit->second,vid1);
      if(w == -1)
        continue;
      DefaultWeight<Cfg> weight;
      weight.SetWeight(w);
      m_highLevelGraph->AddEdge(vit->second,vid1,weight);
    }
  }


  m_hlgTaskVIDs[_wholeTask] = m_currentTaskVIDs;

  //AddTaskToAvailableIntervalGraph(_wholeTask);
  if(m_debug){
    std::cout << "Adding Task." << std::endl;
    PrintGraph();
    //PrintAvailabilityGraph();
  }
  //return std::pair<size_t,size_t>(m_intervalMap[virtStart][0],m_intervalMap[virtGoal][0]);
  //return std::pair<size_t,size_t>(virtStart,virtGoal);
  return m_taskQuery[_wholeTask];
}

void
MultiTaskGraph::
RemoveTaskFromGraph(WholeTask* _wholeTask){
/*
  for(auto& vid : m_currentTaskVIDs){
    //m_highLevelGraph->DeleteVertex(vid);
    m_highLevelGraph->SetVertexInvalidated(vid);
  }
*/
/*
  for(auto vid : m_currentTaskVIDs) {
    for(auto v : m_intervalMap[vid]) {
      //m_availableIntervalGraph->DeleteVertex(v);
      m_availableIntervalGraph->SetVertexInvalidated(v);
    }
    //m_intervalMap.erase(vid);
    //m_agentAvailableIntervalMap.erase(vid);
  }
  for(auto vid : m_hlgTaskVIDs[_wholeTask]) {
    m_highLevelGraph->SetVertexInvalidated(vid);
  }
  for(auto vid : m_aigTaskVIDs[_wholeTask]) {
    m_availableIntervalGraph->SetVertexInvalidated(vid);
  }*/
  //TODO::create the interval map and update it with changes to the rat
  //and task start goal instead of starting from 0
  //m_intervalMap.clear();
  //m_parentIntervalMap.clear();
  //m_agentAvailableIntervalMap.clear();

  //m_goalDelivering.clear();

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

  //return solution->GetPath()->Length();
  return solution->GetPath()->TimeSteps();
}

/*------------------------------ Construction Helpers --------------------------------*/

void
MultiTaskGraph::
ConstructGraph(){
  CombinedRoadmap::ConstructGraph();
  CreateHighLevelGraph();

	m_nonTaskNodes = m_highLevelGraph->Size();

  //CreateAvailableIntervalGraph();
  if(m_debug){
    std::cout << "Initial construction." << std::endl;
    PrintGraph();
  }
}

void
MultiTaskGraph::
CreateHighLevelGraph(){

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  m_highLevelGraph = std::shared_ptr<TaskGraph>(new TaskGraph(robot));

  for(auto& it : this->GetTaskPlan()->GetInteractionTemplates()){
    for(auto pair : it->GetTransformedPositionPairs()){
      auto cfgR = pair.first;//receiving
      auto cfgD = pair.second;//delivering
			if(this->m_discrete) {
				int x = int(cfgR[0]+.5);
				int y = int(cfgR[1] + .5);
				cfgR.SetData({double(x), double(y), 0});
				x = int(cfgD[0]+.5);
				y = int(cfgD[1] + .5);
				cfgD.SetData({double(x), double(y), 0});
			}

      auto vidR = m_highLevelGraph->AddVertex(cfgR);
      auto vidD = m_highLevelGraph->AddVertex(cfgD);

      m_deliveringVIDs[cfgD.GetRobot()->GetCapability()].push_back(vidD);
      m_mainDelivering.insert(vidD);
      m_receivingVIDs[cfgR.GetRobot()->GetCapability()].push_back(vidR);

      DefaultWeight<Cfg> interactionEdge;
      interactionEdge.SetWeight(it->GetInformation()->GetInteractionWeight());

      m_highLevelGraph->AddEdge(vidD,vidR,interactionEdge);
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
/*
  DefaultWeight<Cfg> virtWeight;
  virtWeight.SetWeight(-1);

  for(auto& it : this->GetTaskPlan()->GetInteractionTemplates()){
    for(auto pair : it->GetTransformedPositionPairs()){
      auto cfgR = pair.first;//receiving
      auto cfgD = pair.second;//delivering
      auto vidR = m_highLevelGraph->AddVertex(cfgR);
      auto vidD = m_highLevelGraph->AddVertex(cfgD);

      m_deliveringVIDs[cfgD.GetRobot()->GetCapability()].push_back(vidD);
      m_receivingVIDs[cfgR.GetRobot()->GetCapability()].push_back(vidR);

      //TODO::Change this to just copy over the x,y,z position of the cfg for virtual superRobot
      //auto virtCfg = pair.first;
      //virtCfg.SetRobot(robot);
      //auto virtVID = m_highLevelGraph->AddVertex(virtCfg);

      //m_virtualVIDs[cfgR.GetRobot()->GetCapability()].push_back(virtVID);

      DefaultWeight<Cfg> interactionWeight;
      interactionWeight.SetWeight(it->GetInformation()->GetInteractionWeight());
      //m_highLevelGraph->AddEdge(vidD,virtVID,interactionWeight);
      //m_highLevelGraph->AddEdge(virtVID,vidD,interactionWeight);
      m_highLevelGraph->AddEdge(vidD,vidR,interactionWeight);

      //DefaultWeight<Cfg> virtWeight;
      //virtWeight.SetWeight(-1);
      //m_highLevelGraph->AddEdge(virtVID,vidR,virtWeight);
      //m_highLevelGraph->AddEdge(vidR,virtVID,virtWeight);
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
        //m_highLevelGraph->AddEdge(vidD, vidR, weight);

        auto robotSelectionNode = m_highLevelGraph->GetVertex(vidR);
        robotSelectionNode.SetRobot(robot);
        auto robotSelectionVID = m_highLevelGraph->AddDuplicateVertex(robotSelectionNode);

        m_virtualVIDs.insert(robotSelectionVID);

        m_highLevelGraph->AddEdge(vidR,robotSelectionVID,weight);
        m_highLevelGraph->AddEdge(robotSelectionVID,vidD,virtWeight);
      }
    }
  }
  */
}
/*
std::unordered_map<Agent*,std::vector<std::pair<double,OccupiedInterval>>>
MultiTaskGraph::
GetAgentAvailableIntervals(size_t _source, size_t _target) {//,
  //std::unordered_map<Agent*,std::list<OccupiedInterval>> _RATCache){
  //Check if safe intervals for this node have already been computed and cached
  if(!m_agentSafeIntervalMap[_source].empty())
    return m_agentSafeIntervalMap[_source];

  auto sourceCfg = m_highLevelGraph->GetVertex(_source);
  auto targetCfg = m_highLevelGraph->GetVertex(_target);
  sourceCfg.SetRobot(targetCfg.GetRobot());

  for(auto agentRAT : this->GetTaskPlan()->GetRAT()){
    if(agentRAT.first->GetRobot()->GetCapability() != targetCfg.GetRobot()->GetCapability())
      continue;

    //std::list<OccupiedInterval> cache = _RATCache[agentRAT.first];
    auto intervals = this->GetTaskPlan()->GetRobotAvailability(agentRAT.first);
    //if(!cache.empty())
    //	intervals.merge(cache);

    for(auto iter = intervals.begin(); iter != intervals.end(); iter++){
      auto arrivalTime = iter->GetEndTime() + LowLevelGraphPathWeight(iter->GetEndLocation(),sourceCfg);

      auto next = iter;
      next++;

      if(next != intervals.end()){
        auto safe = std::make_pair(arrivalTime,*next);
        m_agentSafeIntervalMap[_source][agentRAT.first].push_back(safe);
      }
      else {
        Cfg temp(agentRAT.first->GetRobot());
        OccupiedInterval oi(static_cast<HandoffAgent*>(agentRAT.first),
            temp, temp, MAX_DBL, MAX_DBL);
        auto safe = std::make_pair(arrivalTime,oi);
        m_agentSafeIntervalMap[_source][agentRAT.first].push_back(safe);
      }
    }
  }
  return m_agentSafeIntervalMap[_source];
}
*/

bool
MultiTaskGraph::
ValidIntervalEdge(size_t _source, size_t _target, double _edge) {
  auto sourceInterval = m_agentAvailableIntervalMap.at(_source);
  auto targetInterval = m_agentAvailableIntervalMap.at(_target);

  //If the edge is moving between ITs or start/goal then it should be the same
  //robot. If the edge is across an IT or a virtual start/goal then it can be
  //different.
  if(_source == 166 or _target == 166)
		std::cout <<"Right here" <<std::endl;
  if(sourceInterval.first != targetInterval.first and
     (m_mainDelivering.count(m_parentIntervalMap[_target])
      or m_goalDelivering.count(m_parentIntervalMap[_target])))
    return false;

  if(sourceInterval.second.first + _edge > targetInterval.second.second)
    return false;

  if(sourceInterval.second.second < targetInterval.second.first)
    return false;

  return true;
}

void
MultiTaskGraph::
CreateAvailableIntervalGraph(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  m_availableIntervalGraph = std::shared_ptr<AvailableIntervalGraph>(
                             new AvailableIntervalGraph(robot));


  //Add a vertex for every interval location pair
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    auto cfg = vit->property();
    if(cfg.GetRobot() == robot){
      double startTime = 0;
      double endTime = MAX_DBL;

      auto vid = m_availableIntervalGraph->AddDuplicateVertex(cfg);
      m_agentAvailableIntervalMap[vid] = std::make_pair(robot->GetAgent(),
                                         std::make_pair(startTime,endTime));
      m_intervalMap[vit->descriptor()].push_back(vid);
      m_parentIntervalMap[vid] = vit->descriptor();
    }
    for(auto agentRAT : this->GetTaskPlan()->GetRAT()) {
      auto agent = agentRAT.first;
      if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
        continue;
      std::list<OccupiedInterval> rat = agentRAT.second;
      for(auto iter = rat.begin(); iter != rat.end(); iter++) {
				double setupTime = LowLevelGraphPathWeight(iter->GetEndLocation(),cfg);
		
				if(setupTime == -1)
					break;
	
        double startTime = iter->GetEndTime() + setupTime; 

        auto next = iter;
        next++;

        double endTime;
        if(next == rat.end())
          endTime = MAX_DBL;
        else {
          endTime = next->GetStartTime()
                  - LowLevelGraphPathWeight(cfg,next->GetStartLocation());
        }
        cfg.SetRobot(agent->GetRobot());
        auto vid = m_availableIntervalGraph->AddDuplicateVertex(cfg);
        m_agentAvailableIntervalMap[vid] = std::make_pair(agent,
                                           std::make_pair(startTime,endTime));
        m_intervalMap[vit->descriptor()].push_back(vid);
        m_parentIntervalMap[vid] = vit->descriptor();
      }
    }
  }
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : m_intervalMap[eit->source()]) {
        for(auto t : m_intervalMap[eit->target()]) {
          if(ValidIntervalEdge(s,t,eit->property().GetWeight()))
            m_availableIntervalGraph->AddEdge(s,t,eit->property());
        }
      }
    }
  }

  /*
  std::unordered_map<size_t,std::vector<size_t>> intervalMap;

  OccupiedInterval infinite;
  infinite.SetStartTime(MAX_DBL);

  AvailableNode physicalNode;
  physicalNode.m_availableTime = 0;
  physicalNode.m_nextAssignment = infinite;

  //Add a vertex for every location/interval pair
  //Robot selection nodes have intervals corresponding to availability
  //Other nodes have an infinite interval that should always be valid
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    auto vid = vit->descriptor();
    if(m_virtualVIDs.count(vid)) {//robot selection node and needs real intervals
      auto eit = vit->begin();
      auto other = (eit->target() == vid) ? eit->source() : eit->target();
      auto availableIntervals = GetAgentAvailableIntervals(vid,other);
      intervalMap[vid] = {};
      AvailableNode node;
      node.m_vid = vid;
      for(auto agentIntervals : availableIntervals){
        for(auto interval : agentIntervals.second){
          node.m_availableTime = interval.first;
          node.m_nextAssignment = interval.second;
          auto newVID = m_availableIntervalGraph->AddDuplicateVertex(vit->property());

          m_availableIntervalMap[newVID] = node;
          intervalMap[vid].push_back(newVID);
        }
      }
    }
    else {//physical location node - use infinite interval
      physicalNode.m_vid = vid;
      auto newVID = m_availableIntervalGraph->AddDuplicateVertex(vit->property());
      intervalMap[vid] = {newVID};
      m_availableIntervalMap[newVID] = physicalNode;
    }
  }
  //Add edges between vertices
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : intervalMap[eit->source()]) {
        for(auto t : intervalMap[eit->target()]) {
          m_availableIntervalGraph->AddEdge(s,t,eit->property());
        }
      }
    }
  }
  */

}

/********************************* Debug ******************************************/

void
MultiTaskGraph::
PrintGraph(){
  std::cout << "Printing high level vertices:" << std::endl;
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++){
    std::cout << vit->descriptor()
              << " : "
              << vit->property().PrettyPrint()
              << " : "
              << vit->property().GetRobot()->GetLabel()
              << std::endl;
  }
  std::cout << "Printing high level edges:" << std::endl;
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++){
    for(auto eit = vit->begin(); eit != vit->end(); eit++){
      std::cout << eit->source()
                << " -> "
                << eit->target()
                << " : "
                << eit->property().GetWeight()
                << std::endl;
    }
  }
  std::cout << "Robot Availability Table:" << std::endl;
  for(auto& ra : this->GetTaskPlan()->GetRAT()){
    std::cout << ra.first << std::endl;
    for(auto inter : ra.second) {
      std::cout << inter.Print() << std::endl;
    }
  }
}

void
MultiTaskGraph::
PrintAvailabilityGraph(){
  std::cout << "Printing availability vertices" << std::endl;
  for(auto vit = m_availableIntervalGraph->begin(); vit != m_availableIntervalGraph->end(); vit++) {
		//if(m_availableIntervalGraph->IsVertexInvalidated(vit->descriptor()))
		//	continue;
    std::cout << vit->descriptor()
              << " : "
              << vit->property().PrettyPrint()
              << " : "
              << m_agentAvailableIntervalMap[vit->descriptor()].first->GetRobot()->GetLabel()
              << " During "
              << m_agentAvailableIntervalMap[vit->descriptor()].second.first
              << " -> "
              << m_agentAvailableIntervalMap[vit->descriptor()].second.second
              << std::endl;
  }
  std::cout << "Printing availability edges" << std::endl;
  for(auto vit = m_availableIntervalGraph->begin(); vit != m_availableIntervalGraph->end(); vit++) {
		//if(m_availableIntervalGraph->IsVertexInvalidated(vit->descriptor()))
		//	continue;
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			//if(m_availableIntervalGraph->IsVertexInvalidated(eit->target()) or
			//	 m_availableIntervalGraph->IsVertexInvalidated(eit->source()))
			//	continue;
      std::cout << eit->source()
                << " -> "
                << eit->target()
                << " : "
                << eit->property().GetWeight()
                << std::endl;
    }
  }
}

std::pair<std::vector<size_t>,std::vector<size_t>>
MultiTaskGraph::
UpdateAvailableIntervalGraph(std::unordered_map<Agent*,
                             std::vector<OccupiedInterval>> _updates,
														 WholeTask* _wholeTask,
														 std::set<size_t> _validVIDs) {

	//if(_wholeTask != m_currentTask)
		AddTaskToGraph(_wholeTask,_validVIDs);

	if(m_debug) {
		std::cout << "Pre-updating availability graph" << std::endl;
		PrintAvailabilityGraph();
	}
  std::vector<size_t> invalid;
  std::vector<size_t> newValid;
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    std::unordered_map<Agent*,std::vector<std::pair<double,double>>> busyIntervalMap;
    auto cfg = vit->property();
    for(auto kv : _updates) {
      auto agent = kv.first;
			if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
				continue;
      for(auto update : kv.second) {
        double startTime = update.GetStartTime()
                         - LowLevelGraphPathWeight(cfg,update.GetStartLocation());
        double endTime = update.GetEndTime()
                         + LowLevelGraphPathWeight(update.GetEndLocation(),cfg);
        std::pair<double,double> busyInterval = std::make_pair(startTime,endTime);
        busyIntervalMap[agent].push_back(busyInterval);
      }
    }
		size_t desc = vit->descriptor();//gdb - debug purposes
		size_t size = m_intervalMap[desc].size();
    for(size_t i = 0; i < size; i++) {
			auto vid = m_intervalMap[desc][i];
			//if(m_availableIntervalGraph->IsVertexInvalidated(vid))
			//	continue;
      auto agentAvail = m_agentAvailableIntervalMap.at(vid);
      auto agent = agentAvail.first;
			if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
				continue;
      for(auto busyInterval : busyIntervalMap[agent]) {
        const auto& availInterval = agentAvail.second;

        if(busyInterval.first > availInterval.second or
           busyInterval.second < availInterval.first)
          continue; // no overlap
        else if(availInterval.first >= busyInterval.first and
                availInterval.second <= busyInterval.second) {
          //Busy Interval complete encompasses availInterval.
          //Remove the availInterval
          //Change to just mark invalid
          //m_availableIntervalGraph->DeleteVertex(vid);
          invalid.push_back(vid);
					//m_availableIntervalGraph->SetVertexInvalidated(vid);
          continue;
        }
        else if(availInterval.first < busyInterval.first and
                availInterval.second > busyInterval.second) {
          //Avail interval completely encompasses busy interval.
          //Split it into two avail intervals
          //Change to mark invalid
          //m_availableIntervalGraph->DeleteVertex(vid);
          invalid.push_back(vid);
					//m_availableIntervalGraph->SetVertexInvalidated(vid);

          auto firstInterval = std::make_pair(availInterval.first,busyInterval.first);
          auto secondInterval = std::make_pair(busyInterval.second,availInterval.second);

					if(firstInterval.first > firstInterval.second)
						throw RunTimeException(WHERE, "This is bad.");
					if(secondInterval.first > secondInterval.second)
						throw RunTimeException(WHERE, "This is bad.");

          auto firstVID = m_availableIntervalGraph->AddDuplicateVertex(cfg);
          auto secondVID = m_availableIntervalGraph->AddDuplicateVertex(cfg);

          m_intervalMap[vit->descriptor()].push_back(firstVID);
          m_parentIntervalMap[firstVID] = vit->descriptor();
          m_intervalMap[vit->descriptor()].push_back(secondVID);
          m_parentIntervalMap[secondVID] = vit->descriptor();

          m_agentAvailableIntervalMap[firstVID] = std::make_pair(agent,firstInterval);
          m_agentAvailableIntervalMap[secondVID] = std::make_pair(agent,secondInterval);

					m_aigTaskVIDs[_wholeTask].push_back(firstVID);
					m_aigTaskVIDs[_wholeTask].push_back(secondVID);

          newValid.push_back(firstVID);
          newValid.push_back(secondVID);

					/*if(m_mainDelivering.count(vid)){
						m_mainDelivering.insert(firstVID);
						m_mainDelivering.insert(secondVID);
					}
					else if(m_goalDelivering.count(vid)){
						m_goalDelivering.insert(firstVID);
						m_goalDelivering.insert(secondVID);
					}*/
          continue;
        }
        else if(availInterval.first < busyInterval.second and 
								availInterval.first >= busyInterval.first) {
          //Busy Interval cuts out beginning of avail interval
          //Change to just marking invalid
          //m_availableIntervalGraph->DeleteVertex(vid);
          invalid.push_back(vid);
					//m_availableIntervalGraph->SetVertexInvalidated(vid);

          auto newInterval = std::make_pair(busyInterval.second,availInterval.second);
					if(newInterval.first > newInterval.second)
						throw RunTimeException(WHERE, "This is bad.");

          auto newVID = m_availableIntervalGraph->AddDuplicateVertex(cfg);

          m_intervalMap[vit->descriptor()].push_back(newVID);
          m_parentIntervalMap[newVID] = vit->descriptor();
          m_agentAvailableIntervalMap[newVID] = std::make_pair(agent,newInterval);

					m_aigTaskVIDs[_wholeTask].push_back(newVID);

          newValid.push_back(newVID);

					if(m_mainDelivering.count(vid)){
						m_mainDelivering.insert(newVID);
					}/*
					else if(m_goalDelivering.count(vid)){
						m_goalDelivering.insert(newVID);
					}*/
          continue;
        }
        else if(availInterval.second > busyInterval.first and
								availInterval.second <= busyInterval.second) {
          //Busy Interval cuts out the end of the available interval
          //Change to just markinginvalid
          //m_availableIntervalGraph->DeleteVertex(vid);
          invalid.push_back(vid);
					//m_availableIntervalGraph->SetVertexInvalidated(vid);

          auto newInterval = std::make_pair(availInterval.first,busyInterval.first);
					if(newInterval.first > newInterval.second)
						throw RunTimeException(WHERE, "This is bad.");

          auto newVID = m_availableIntervalGraph->AddDuplicateVertex(cfg);

          m_intervalMap[vit->descriptor()].push_back(newVID);
          m_parentIntervalMap[newVID] = vit->descriptor();
          m_agentAvailableIntervalMap[newVID] = std::make_pair(agent,newInterval);

					m_aigTaskVIDs[_wholeTask].push_back(newVID);

          newValid.push_back(newVID);
          continue;
        }
				else if(availInterval.first != busyInterval.second and 
								availInterval.second != busyInterval.first) {
					std::cout << "AVIAL: " << availInterval << std::endl;
					std::cout << "BUSY: " << busyInterval << std::endl;
        	throw RunTimeException(WHERE,"Unaccounted for condition.");
				}
      }
    }
  }
  //TODO::Add edges
/*
  for(auto vid : newValid) {
    TaskGraph::CVI vit;
    m_highLevelGraph->IsVertex(m_highLevelGraph->GetVertex(m_parentIntervalMap[vid]),vit);
    if(vit == m_highLevelGraph->end())
      throw RunTimeException(WHERE,"This should not happen.");
	*/
	std::set<size_t> newValidSet;
	for(auto v : newValid) {
		newValidSet.insert(v);
	}
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
		//size_t vid = vit->descriptor();
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : m_intervalMap[eit->source()]) {
        for(auto t : m_intervalMap[eit->target()]) {
          //if(!(s == vid or t ==vid))
          //  continue;
          if(!newValidSet.count(s) and !newValidSet.count(t))
						continue;
					if((vit->descriptor() == 31 or vit->descriptor() == 29) and (s == 166 or t ==166))
									std::cout << "Also right here." << std::endl;
					//if(m_availableIntervalGraph->IsVertexInvalidated(s) or
					//	 m_availableIntervalGraph->IsVertexInvalidated(t))
					//	continue;
          if(ValidIntervalEdge(s,t,eit->property().GetWeight()))
            m_availableIntervalGraph->AddEdge(s,t,eit->property());
        }
      }
    }
  }
	if(m_debug) {
		std::cout << "Post updating availability graph" << std::endl;
		PrintAvailabilityGraph();
	}
  return std::make_pair(invalid,newValid);
}

void
MultiTaskGraph::
AddTaskToAvailableIntervalGraph(WholeTask* _wholeTask) {

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  //for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
  for(auto v : m_currentTaskVIDs) {
    /*TaskGraph::CVI vit;
    m_highLevelGraph->IsVertex(m_highLevelGraph->GetVertex(v),vit);
    if(vit == m_highLevelGraph->end())
      throw RunTimeException(WHERE,"This is a problem and shouldn't happen.");
		*/
		auto vit = m_highLevelGraph->begin();
		while(vit!= m_highLevelGraph->end()) {
			if(vit->descriptor() == v)
				break;
			vit++;
		}
		if(vit==m_highLevelGraph->end())
      throw RunTimeException(WHERE,"This is a problem and shouldn't happen.");

    auto cfg = m_highLevelGraph->GetVertex(v);
    if(cfg.GetRobot() == robot){
      double startTime = 0;
      double endTime = MAX_DBL;

      auto vid = m_availableIntervalGraph->AddDuplicateVertex(cfg);
      m_agentAvailableIntervalMap[vid] = std::make_pair(robot->GetAgent(),
                                         std::make_pair(startTime,endTime));
      m_intervalMap[vit->descriptor()].push_back(vid);
      m_parentIntervalMap[vid] = vit->descriptor();
			m_aigTaskVIDs[_wholeTask].push_back(vid);
			continue;
    }
    for(auto agentRAT : this->GetTaskPlan()->GetRAT()) {
      auto agent = agentRAT.first;
      if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
        continue;
      std::list<OccupiedInterval> rat = agentRAT.second;
      for(auto iter = rat.begin(); iter != rat.end(); iter++) {
        double startTime = iter->GetEndTime()
                         + LowLevelGraphPathWeight(iter->GetEndLocation(),cfg);

        auto next = iter;
        next++;

        double endTime;
        if(next == rat.end())
          endTime = MAX_DBL;
        else {
          endTime = next->GetStartTime()
                  - LowLevelGraphPathWeight(cfg,next->GetStartLocation());
        }
        cfg.SetRobot(agent->GetRobot());
        auto vid = m_availableIntervalGraph->AddDuplicateVertex(cfg);
        m_agentAvailableIntervalMap[vid] = std::make_pair(agent,
                                           std::make_pair(startTime,endTime));
        m_intervalMap[vit->descriptor()].push_back(vid);
        m_parentIntervalMap[vid] = vit->descriptor();

        m_aigTaskVIDs[_wholeTask].push_back(vid);
      }
    }
  }
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
  //THIS IS ONLY CHECKING THE TASK TO TASK VERTICES AND NOT CHECKING THE TASK TO MAIN GRAPH VERTICES
  /*for(auto v : m_currentTaskVIDs) {
    TaskGraph::CVI vit;
    m_highLevelGraph->IsVertex(m_highLevelGraph->GetVertex(v),vit);
    if(vit == m_highLevelGraph->end())
      throw RunTimeException(WHERE,"This is a problem and shouldn't happen.");*/
		size_t vid = vit->descriptor();
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : m_intervalMap[eit->source()]) {
        for(auto t : m_intervalMap[eit->target()]) {
          if(eit->target() == 15 or eit->source() == 8)
            std::cout << "HERE" << vit->descriptor() << std::endl << "VID: " << vid << std::endl;
          if(ValidIntervalEdge(s,t,eit->property().GetWeight()))
            m_availableIntervalGraph->AddEdge(s,t,eit->property());
        }
      }
    }
  }
}

std::unordered_map<WholeTask*,std::vector<size_t>>&
MultiTaskGraph::
GetTaskAigVIDs() {
  return m_aigTaskVIDs;
}
		
size_t 
MultiTaskGraph::
NonTaskNodes() {
	return m_nonTaskNodes;
}
