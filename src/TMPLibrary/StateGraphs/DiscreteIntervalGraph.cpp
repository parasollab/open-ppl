#include "DiscreteIntervalGraph.h"

#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/TaskPlan.h"

/************************************ Construction *************************************/

DiscreteIntervalGraph::
DiscreteIntervalGraph(){
  this->SetName("DiscreteIntervalGraph");
}

DiscreteIntervalGraph::
DiscreteIntervalGraph(XMLNode& _node) : CombinedRoadmap(_node){
  this->SetName("DiscreteIntervalGraph");
}

/************************************ Initialization *************************************/

void
DiscreteIntervalGraph::
Initialize(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  m_highLevelGraph = std::shared_ptr<TaskGraph>(new TaskGraph(robot));
  CombinedRoadmap::Initialize();
}

/*************************************** Accessors ***************************************/

RoadmapGraph<Cfg, DefaultWeight<Cfg>>*
DiscreteIntervalGraph::
GetGraph(){
  return m_highLevelGraph.get();
}

std::shared_ptr<RoadmapGraph<Cfg,DefaultWeight<Cfg>>>
DiscreteIntervalGraph::
GetAvailableIntervalGraph() {
  return m_availableIntervalGraph;
}

std::pair<Agent*,std::pair<size_t,size_t>>
DiscreteIntervalGraph::
GetInterval(size_t _vid){
  return m_agentAvailableIntervalMap.at(_vid);
}

size_t
DiscreteIntervalGraph::
ValidTransition(size_t _source, size_t _target, size_t _edge,
                size_t _sourceDistance, std::pair<size_t,Cfg> _update, ConstraintMap _constraints) {
  //if(_update.first == 0)
  //  return _edge;
  auto sourceInterval = m_agentAvailableIntervalMap.at(_source);

  if(_sourceDistance > sourceInterval.second.second or
     _sourceDistance < sourceInterval.second.first)
    return MAX_INT;

	/*if(_target == MAX_INT and _edge == MAX_INT) {
		return 0;
	}*/

  auto targetInterval = m_agentAvailableIntervalMap.at(_target);

  auto sourceCfg = m_availableIntervalGraph->GetVertex(_source);
  auto targetCfg = m_availableIntervalGraph->GetVertex(_target);

	size_t transitionWeight = _edge;

	if(sourceCfg.GetRobot()->GetCapability() == targetCfg.GetRobot()->GetCapability()) {
		auto path = LowLevelGraphPath(sourceCfg,targetCfg,_constraints,_sourceDistance+1);
		if(path.size() == 0)
			return MAX_INT;
		//if(path.size() == 1)
		//	transitionWeight = 0;
		//else
		transitionWeight = path.size();
		//if(sourceCfg != targetCfg)
		//auto hlSource = m_parentIntervalmap[_source];
		auto hlTarget = m_parentIntervalMap[_target];
		if((m_mainDelivering.count(hlTarget) or m_goalDelivering.count(hlTarget)) and sourceCfg != targetCfg)
			transitionWeight = path.size()-1;
	}

  //if(_sourceDistance + _edge > targetInterval.second.second)// or
  if(_sourceDistance + transitionWeight > targetInterval.second.second)// or
     //_sourceDistance + _edge < targetInterval.second.first)//dont use bc you can wait
    return MAX_INT;

  //Subtask start and end
  if(sourceCfg.GetRobot() == targetCfg.GetRobot())
    //return _edge;
    return transitionWeight;
  //Interaction and transition between subtasks - may have waiting time
  //TODO::Figure out how to update the transition time when considering path constraints
  size_t availTime;
  if(_update.first == 0){
    availTime = targetInterval.second.first;
    /*Cfg startCfg = this->GetTaskPlan()->GetRobotAvailability(
										targetCfg.GetRobot()->GetAgent()).front().GetEndLocation();

		int x = int(startCfg[0]+.5);
		int y = int(startCfg[1]+.5);

		startCfg.SetData({double(x),double(y),0});
		startCfg.SetRobot(targetCfg.GetRobot());

    availTime = _update.first
								+ LowLevelGraphPathWeight(startCfg,targetCfg,_constraints,0);*/
	}
  else {
    availTime = _update.first
								+ LowLevelGraphPathWeight(_update.second,targetCfg,_constraints,_update.first+1);
		if(availTime == _update.first)
			return MAX_INT;
    //          + LowLevelGraphPathWeight(_update.second,targetCfg);
	}


  //Check if the extra waiting time violates the delivering or receiving
  //interval
  if(availTime > targetInterval.second.second or
     availTime > sourceInterval.second.second)
    return MAX_INT;

  //TODO::I think this is implicitly done by failing the next check anyways
  //Check if there will be no waiting time
  //if(availTime < targetInterval.second.first)
  //  return _edge;//should be 0 unless bias is changed

  //Check if the delivering robot will need to wait
  //if(availTime > _sourceDistance + _edge)
  if(availTime > _sourceDistance + transitionWeight)
    return availTime - _sourceDistance;

  return transitionWeight;
}

size_t
DiscreteIntervalGraph::
ExtractPathWeight(size_t _vid1, size_t _vid2, bool _forceMatch, ConstraintMap _constraints,
						size_t _startTime, size_t _minEndTime) {

	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	/*if(_forceMatch){
		if(start.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			start.SetRobot(goal.GetRobot());
		else if (goal.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			goal.SetRobot(start.GetRobot());
	}
	*/
	return LowLevelGraphPathWeight(start,goal,_constraints, _startTime, _minEndTime);
}

std::vector<size_t>
DiscreteIntervalGraph::
ExtractPath(size_t _vid1, size_t _vid2, bool _forceMatch, ConstraintMap _constraints,
						size_t _startTime, size_t _minEndTime) {
	auto start = m_highLevelGraph->GetVertex(_vid1);
	auto goal = m_highLevelGraph->GetVertex(_vid2);

	/*if(_forceMatch){
		if(start.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			start.SetRobot(goal.GetRobot());
		else if (goal.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot())
			goal.SetRobot(start.GetRobot());
	}
	*/
	return LowLevelGraphPath(start,goal,_constraints,_startTime,_minEndTime);
}

std::pair<size_t,size_t>
DiscreteIntervalGraph::
AddTaskToGraph(WholeTask* _wholeTask,std::set<size_t> _validAigVIDs) {

  /*if(m_currentTask) {
    for(auto vid : m_hlgTaskVIDs[m_currentTask]) {
      m_highLevelGraph->SetVertexInvalidated(vid);
    }
    for(auto vid : m_aigTaskVIDs[m_currentTask]) {
      m_availableIntervalGraph->SetVertexInvalidated(vid);
    }
  }*/
  m_currentTask = _wholeTask;
  if(!m_hlgTaskVIDs[_wholeTask].empty()) {
		/*
    for(auto vid : m_hlgTaskVIDs[_wholeTask]) {
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
		*/
  	if(m_debug){
    	std::cout << "Adding Task." << std::endl;
    	PrintGraph();
    	PrintAvailabilityGraph();
  	}
    //return std::pair<size_t,size_t>(m_intervalMap[m_aigTaskVIDs[_wholeTask][0]][0],
    //                                m_intervalMap[m_aigTaskVIDs[_wholeTask][1]][0]);
    return std::pair<size_t,size_t>(m_aigTaskVIDs[_wholeTask][0],
                                    m_aigTaskVIDs[_wholeTask][1]);
  }

  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  //TODO::Put a check here to make sure these are valid cfgs and already exist in the map
  Cfg start = _wholeTask->m_startPoints[robot->GetLabel()][0];
  Cfg goal = _wholeTask->m_goalPoints[robot->GetLabel()][0];

	int x = int(start[0]+.5);
	int y = int(start[1]+.5);
	start.SetData({double(x),double(y),0});
	x = int(goal[0]+.5);
	y = int(goal[1]+.5);
	goal.SetData({double(x),double(y),0});

  auto virtStart = m_highLevelGraph->AddDuplicateVertex(start);
  auto virtGoal = m_highLevelGraph->AddDuplicateVertex(goal);

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
		x = int(cfg[0]+.5);
		y = int(cfg[1]+.5);
		cfg.SetData({double(x),double(y),0});
    auto vid1 = m_highLevelGraph->AddDuplicateVertex(cfg);
    m_currentTaskVIDs.push_back(vid1);
    startVIDs[cfg.GetRobot()->GetCapability()] = vid1;
    //Connect robot-type start node to the virtual start node
    m_highLevelGraph->AddEdge(virtStart,vid1,startEdge);

    for(auto& vid2 : m_deliveringVIDs[cfg.GetRobot()->GetCapability()]){
      //auto w = ExtractPathWeight(vid1,vid2);
      //if(w == MAX_INT)
      //  continue;
      auto path = ExtractPath(vid1,vid2);
			if(path.empty())
				continue;
      DefaultWeight<Cfg> weight;
      weight.SetWeight(path.size());
      m_highLevelGraph->AddEdge(vid1,vid2,weight);
			m_highLevelEdgePaths[vid1][vid1] = path;
    }
  }
  //Add robot-type goal nodes
  for(auto& pair : _wholeTask->m_goalPoints){
    auto cfg = pair.second[0];
    if(cfg.GetRobot() == robot)
      continue;
		x = int(cfg[0]+.5);
		y = int(cfg[1]+.5);
		cfg.SetData({double(x),double(y),0});
    auto vid1 = m_highLevelGraph->AddDuplicateVertex(cfg);
    m_currentTaskVIDs.push_back(vid1);
    m_goalDelivering.insert(vid1);
    //Connect robot-type goal node to the virtual goal node
    m_highLevelGraph->AddEdge(vid1,virtGoal,startEdge);
    //m_highLevelGraph->AddEdge(virtGoal,vid1,virtEdge);

    for(auto& vid2 : m_receivingVIDs[cfg.GetRobot()->GetCapability()]){
      //auto w = ExtractPathWeight(vid2,vid1);
      //if(w == MAX_INT)
      //  continue;
      auto path = ExtractPath(vid2,vid1);
			if(path.empty())
				continue;
      DefaultWeight<Cfg> weight;
      weight.SetWeight(path.size());
      m_highLevelGraph->AddEdge(vid2,vid1,weight);
			m_highLevelEdgePaths[vid2][vid1] = path;
    }
    //Attempt to directly connect the goal to the start
    auto vit = startVIDs.find(cfg.GetRobot()->GetCapability());
    if(vit != startVIDs.end()){
      //auto w = ExtractPathWeight(vit->second,vid1);
      //if(w == MAX_INT)
      //  continue;
      auto path = ExtractPath(vit->second,vid1);
			if(path.empty())
				continue;

      DefaultWeight<Cfg> weight;
      weight.SetWeight(path.size());
      m_highLevelGraph->AddEdge(vit->second,vid1,weight);
			m_highLevelEdgePaths[vit->second][vid1] = path;
    }
  }

  m_hlgTaskVIDs[_wholeTask] = m_currentTaskVIDs;

  AddTaskToAvailableIntervalGraph(_wholeTask);
  if(m_debug){
    std::cout << "Adding Task." << std::endl;
    PrintGraph();
    PrintAvailabilityGraph();
  }
	size_t queryStart = 0;
	for(auto vid : m_intervalMap[virtStart]) {
		//if(!m_availableIntervalGraph->IsVertexInvalidated(vid)) {
			queryStart = vid;
			break;
		//}
	}
	size_t queryGoal = 0;
	for(auto vid : m_intervalMap[virtGoal]) {
		//if(!m_availableIntervalGraph->IsVertexInvalidated(vid)) {
			queryGoal = vid;
			break;
		//}
	}
	return std::make_pair(queryStart,queryGoal);
  //return std::pair<size_t,size_t>(m_intervalMap[virtStart][0],m_intervalMap[virtGoal][0]);
}

void
DiscreteIntervalGraph::
RemoveTaskFromGraph(WholeTask* _wholeTask){

  /*for(auto& vid : m_currentTaskVIDs){
    //m_highLevelGraph->DeleteVertex(vid);
    m_highLevelGraph->SetVertexInvalidated(vid);
  }*/
/*
  for(auto vid : m_currentTaskVIDs) {
    for(auto v : m_intervalMap[vid]) {
      //m_availableIntervalGraph->DeleteVertex(v);
      m_availableIntervalGraph->SetVertexInvalidated(v);
    }
    //m_intervalMap.erase(vid);
    //m_agentAvailableIntervalMap.erase(vid);
  }*/
	/*
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
size_t
DiscreteIntervalGraph::
LowLevelGraphPathWeight(Cfg _start, Cfg _goal, ConstraintMap _constraints,
												size_t _startTime, size_t _minEndTime){
	return LowLevelGraphPath(_start,_goal, _constraints, _startTime, _minEndTime).size();
}

std::vector<size_t>
DiscreteIntervalGraph::
LowLevelGraphPath(Cfg _start, Cfg _goal, ConstraintMap _constraints,
												size_t _startTime, size_t _minEndTime){

  if(_start.GetRobot()->GetCapability() != _goal.GetRobot()->GetCapability()){
    throw RunTimeException(WHERE, "start and goal are of mismatched robot types.");
  }

	auto originalAgent = static_cast<HandoffAgent*>(_start.GetRobot()->GetAgent());

  auto dummyAgent = this->GetTaskPlan()->GetCapabilityAgent(_start.GetRobot()->GetCapability());
  auto robot = dummyAgent->GetRobot();

  _start.SetRobot(robot);
  _goal.SetRobot(robot);

	int x = int(_start[0] + 0.5);
	int y = int(_start[1] + 0.5);
	_start.SetData({double(x),double(y),0});
	x = int(_goal[0] + 0.5);
	y = int(_goal[1] + 0.5);
	_goal.SetData({double(x),double(y),0});


	auto roadmap = this->GetCapabilityRoadmap(dummyAgent);
	size_t startVID = roadmap->GetVID(_start);
	size_t goalVID = roadmap->GetVID(_goal);

	return LowLevelGraphPath(originalAgent, startVID, goalVID, _constraints, _startTime, _minEndTime);
}

std::vector<size_t>
DiscreteIntervalGraph::
LowLevelGraphPath(HandoffAgent* _agent, size_t _start, size_t _goal, ConstraintMap _constraints,
												size_t _startTime, size_t _minEndTime){

	if(_minEndTime == 0) {
		_minEndTime = _startTime;
	}

  //auto dummyAgent = this->GetTaskPlan()->GetCapabilityAgent(_agent->GetCapability());
	auto roadmap = this->GetCapabilityRoadmap(_agent);

	size_t maxConstraint = 0;

	std::unordered_map<size_t, std::set<std::pair<size_t,size_t>>> pathConstraints;
	for(auto timeConstraint : _constraints[_agent]) {
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

	bool safeAfterPath = false;

	std::list<std::shared_ptr<Vertex>> pq;
	size_t currentDistance = _startTime+1;
	std::set<size_t> discoveredVertices;
	while(current->m_vid != _goal or current->m_distance < _minEndTime or !safeAfterPath) {

		if(current->m_vid == _goal and current->m_distance >= _minEndTime) {
			if(!pathConstraints[current->m_distance].count(std::make_pair(_goal,MAX_INT))) {
				safeAfterPath = true;
				break;
			}
		}

		if(current->m_distance > roadmap->Size() + std::max(maxConstraint,_minEndTime))
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

				/*if(pq.empty())
					pq.push_back(vert);
				else
					for(auto iter = pq.begin(); iter != pq.end(); iter++) {

					}
				*/
				//Just going to push back since each will increment by one each time
				pq.push_back(vert);
				discoveredVertices.insert(target);
			}
		}

		if(pq.empty())
			return {};

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
}

/*------------------------------ Construction Helpers --------------------------------*/

void
DiscreteIntervalGraph::
ConstructGraph(){
  CombinedRoadmap::ConstructGraph();
  CreateHighLevelGraph();
  CreateAvailableIntervalGraph();
  if(m_debug){
    std::cout << "Initial construction." << std::endl;
    PrintGraph();
  }
}

void
DiscreteIntervalGraph::
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

      auto vidR = m_highLevelGraph->AddDuplicateVertex(cfgR);
      auto vidD = m_highLevelGraph->AddDuplicateVertex(cfgD);

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
        //auto w = ExtractPathWeight(vidR, vidD);
        auto path = ExtractPath(vidR,vidD);
        //if(w == MAX_INT) //indicates there is no path between the two in the lower level graph
        if(path.empty())
          continue;
        DefaultWeight<Cfg> weight;
        weight.SetWeight(path.size());
				m_highLevelEdgePaths[vidR][vidD] = path;
        m_highLevelGraph->AddEdge(vidR, vidD, weight);
      }
    }
  }
}
bool
DiscreteIntervalGraph::
ValidIntervalEdge(size_t _source, size_t _target, size_t _edge) {
  auto sourceInterval = m_agentAvailableIntervalMap.at(_source);
  auto targetInterval = m_agentAvailableIntervalMap.at(_target);

  //If the edge is moving between ITs or start/goal then it should be the same
  //robot. If the edge is across an IT or a virtual start/goal then it can be
  //different.
  //if(sourceInterval.first != targetInterval.first and
     //(m_mainDelivering.count(m_parentIntervalMap[_target])
      //or m_goalDelivering.count(m_parentIntervalMap[_target])))
    //return false;

	//Prevents homogeneous interactions at the moment
	if(sourceInterval.first->GetCapability() == targetInterval.first->GetCapability()
		 and sourceInterval.first != targetInterval.first)
		return false;

  if(sourceInterval.second.first + _edge > targetInterval.second.second)
    return false;

  if(sourceInterval.second.second < targetInterval.second.first)
    return false;

  return true;
}

void
DiscreteIntervalGraph::
CreateAvailableIntervalGraph(){
  auto robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
  m_availableIntervalGraph = std::shared_ptr<AvailableIntervalGraph>(
                             new AvailableIntervalGraph(robot));


  //Add a vertex for every interval location pair
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    auto cfg = vit->property();
    //if(cfg.GetRobot() == robot){
    for(auto agent : this->GetTaskPlan()->GetTeam()) {
			if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
				continue;

			auto allocations = this->GetTaskPlan()->GetRobotAvailability(agent);

			for(auto iter = allocations.begin(); iter != allocations.end(); iter++) {
				auto alloc = *(iter);
      	size_t beginTime = size_t(alloc.GetEndTime());
				if(beginTime > 0)
					beginTime++;

				Cfg beginCfg = alloc.GetStartLocation();
				int x = int(beginCfg[0]+.5);
				int y = int(beginCfg[1]+.5);
				beginCfg.SetData({double(x),double(y),0});
				beginCfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(agent->GetCapability())->GetRobot());

      	size_t arriveTime = LowLevelGraphPathWeight(beginCfg,cfg,{},beginTime);
				size_t availTime = arriveTime;// + 1;
				//if(arriveTime == 0
					 if(m_mainDelivering.count(vit->descriptor())
					 or m_goalDelivering.count(vit->descriptor())) {
					availTime--;
				}

				size_t endTime = MAX_INT;
				auto next = iter;
				next++;
				if(next != allocations.end()) {
					Cfg nextCfg = next->GetStartLocation();
					x = int(nextCfg[0]+.5);
					y = int(nextCfg[1]+.5);
					nextCfg.SetData({double(x),double(y),0});
					nextCfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(agent->GetCapability())->GetRobot());

					endTime = size_t(next->GetStartTime()) - LowLevelGraphPathWeight(cfg,nextCfg);
				}

				cfg.SetRobot(agent->GetRobot());
      	auto vid = m_availableIntervalGraph->AddDuplicateVertex(cfg);
      	m_agentAvailableIntervalMap[vid] = std::make_pair(agent,
                                         //std::make_pair(arriveTime,endTime));
                                         std::make_pair(availTime,endTime));
      	m_intervalMap[vit->descriptor()].push_back(vid);
      	m_parentIntervalMap[vid] = vit->descriptor();
			}
    }
  }
	if(m_debug)
		PrintAvailabilityGraph();
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : m_intervalMap[eit->source()]) {
        for(auto t : m_intervalMap[eit->target()]) {
          if(ValidIntervalEdge(s,t,eit->property().GetWeight())) {
            m_availableIntervalGraph->AddEdge(s,t,eit->property());
						//m_availableIntervalEdgePaths[s][t] = m_highLevelEdgePaths[eit->source()][eit->target()];
					}
        }
      }
    }
  }
	if(m_debug)
		PrintAvailabilityGraph();
}

/********************************* Debug ******************************************/

void
DiscreteIntervalGraph::
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
                << " : ";
			for(auto vid : m_highLevelEdgePaths[eit->source()][eit->target()]) {
				std::cout << vid << "->";
			}
			std::cout << std::endl;
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
DiscreteIntervalGraph::
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
                << " : ";
			for(auto vid : m_availableIntervalEdgePaths[eit->source()][eit->target()]) {
				std::cout << vid << "->";
			}
			std::cout << std::endl;
    }
  }
}

std::pair<std::vector<size_t>,std::vector<size_t>>
DiscreteIntervalGraph::
UpdateAvailableIntervalConstraint(HandoffAgent* _agent, size_t _startTime, size_t _endTime,
																	size_t _startVID, size_t _endVID, WholeTask* _wholeTask,
														 			std::set<size_t> _validVIDs, ConstraintMap _constraints) {

	//if(_wholeTask != m_currentTask)
		AddTaskToGraph(_wholeTask,_validVIDs);

	auto roadmap = this->GetCapabilityRoadmap(_agent);
	auto startCfg = roadmap->GetVertex(_startVID);
	auto endCfg = roadmap->GetVertex(_endVID);


	if(m_debug) {
		std::cout << "Pre-updating availability graph" << std::endl;
		PrintAvailabilityGraph();
	}
  std::vector<size_t> invalid;
  std::vector<size_t> newValid;
  for(auto vit = m_highLevelGraph->begin(); vit != m_highLevelGraph->end(); vit++) {
    std::unordered_map<Agent*,std::vector<std::pair<size_t,size_t>>> busyIntervalMap;
    auto cfg = vit->property();
		if(_agent->GetRobot()->GetCapability() != cfg.GetRobot()->GetCapability())
			continue;
			cfg.SetRobot(_agent->GetRobot());
    //for(auto kv : _updates) {
      //auto agent = kv.first;
			//if(agent->GetCapability() != cfg.GetRobot()->GetCapability())
				//continue;
      //for(auto update : kv.second) {
        size_t setupCost = LowLevelGraphPathWeight(cfg,startCfg);
				//if(setupCost == 1)
				//	setupCost = 0;

				size_t startTime; //acount for unsigned nature of variables
				if(setupCost > _startTime)
					startTime = 0;
				else
					startTime = _startTime - setupCost;

				//size_t startTime = _startTime
        //                 - LowLevelGraphPathWeight(cfg,startCfg);



				auto travelTime = LowLevelGraphPathWeight(endCfg,cfg);
				if(travelTime == 1)
					travelTime = 0;

        size_t endTime = _endTime + travelTime;

        std::pair<size_t,size_t> busyInterval = std::make_pair(startTime,endTime);
        busyIntervalMap[_agent].push_back(busyInterval);
      //}
    //}
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

				if(!m_goalDelivering.count(vit->descriptor()) and !m_mainDelivering.count(vit->descriptor())){
					//busyInterval.first = busyInterval.first - 1;
					busyInterval.second = busyInterval.second + 1;
				}
				else {
					busyInterval.first = busyInterval.first - 1;
				}

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

					/*if(!m_goalDelivering.count(vit->descriptor()) and !m_mainDelivering.count(vit->descriptor())){
						//busyInterval.first = busyInterval.first - 1;
						busyInterval.second = busyInterval.second + 1;
					}
					else {
						busyInterval.first = busyInterval.first - 1;
					}*/

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

					/*if(!m_goalDelivering.count(vit->descriptor()) and !m_mainDelivering.count(vit->descriptor())){
						//busyInterval.first = busyInterval.first - 1;
						busyInterval.second = busyInterval.second + 1;
					}*/
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
					/*if(!m_goalDelivering.count(vit->descriptor()) and !m_mainDelivering.count(vit->descriptor())){
						//busyInterval.first = busyInterval.first - 1;
						busyInterval.second = busyInterval.second + 1;
					}
					else {
						busyInterval.first = busyInterval.first - 1;
					}*/

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
					//if(m_availableIntervalGraph->IsVertexInvalidated(s) or
					//	 m_availableIntervalGraph->IsVertexInvalidated(t))
					//	continue;

  				auto sourceInterval = m_agentAvailableIntervalMap.at(s);
  				auto targetInterval = m_agentAvailableIntervalMap.at(t);
					std::vector<size_t> path;

					auto sourceCfg = m_highLevelGraph->GetVertex(eit->source());
					auto targetCfg = m_highLevelGraph->GetVertex(eit->target());

					if(sourceCfg.GetRobot()->GetCapability() == targetCfg.GetRobot()->GetCapability())
						path = LowLevelGraphPath(sourceCfg,targetCfg,
															 			 _constraints,sourceInterval.second.first,
															 			 targetInterval.second.first);

          if(ValidIntervalEdge(s,t,path.size())) {
						DefaultWeight<Cfg> edge;
						if(!path.empty()) {
							//if(m_goalDelivering.count(eit->target()) or m_mainDelivering.count(eit->target()))
							//	edge.SetWeight(path.size()-1);
							//else
								edge.SetWeight(path.size());
						}
						else
							edge.SetWeight(eit->property().GetWeight());
            m_availableIntervalGraph->AddEdge(s,t,edge);
						//TODO::See if this makes any sense
						//Trying to get the length of the path between the source and target with respect
						//to the contraints. Have to start at the beginning of the sourceInterval and can't finish
						//before the target interval begins
					}
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

std::pair<std::vector<size_t>,std::vector<size_t>>
DiscreteIntervalGraph::
UpdateMotionConstraint(HandoffAgent* _agent,WholeTask* _wholeTask,
											std::set<size_t> _validVIDs, ConstraintMap _constraints) {

	AddTaskToGraph(_wholeTask,_validVIDs);

	std::set<size_t> modifiedVertices;
	std::unordered_map<size_t,size_t> newVertices;


	auto allocations = this->GetTaskPlan()->GetAgentAllocations(_agent);

	for(auto hvit = m_highLevelGraph->begin(); hvit != m_highLevelGraph->end(); hvit++) {
		auto iter = allocations.begin();
		auto next = iter;
		next++;
		for(auto vid : m_intervalMap[hvit->descriptor()]) {
			//if(m_availableIntervalGraph->IsVertexInvalidated(vid))
			//	continue;

			auto agentInterval = m_agentAvailableIntervalMap[vid];
			if(_agent != agentInterval.first)
				continue;
			while(!(agentInterval.second.first > iter->m_endTime and
					 agentInterval.second.second < next->m_startTime) and
					 next != allocations.end()) {
				iter++; next++;
			}
			//if(next == allocations.end()) {
			//	break;
			//}

			std::pair<size_t,size_t> newInterval;

			auto cfg = m_availableIntervalGraph->GetVertex(vid);
			cfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(_agent->GetCapability())->GetRobot());

			auto lowLevelVID = this->GetCapabilityRoadmap(_agent)->GetVID(cfg);

			auto sTime = iter->m_endTime;
			if(sTime != 0) {
				sTime++;
			}

			auto setupPath = LowLevelGraphPath(_agent, iter->m_endLocation,lowLevelVID,
															 			_constraints,
																		//iter->m_endTime,
																		sTime,
																		//agentInterval.second.first);
																		agentInterval.second.first);

			//if(iter->m_endTime + setuPath.size() - 1 != agentInterval.second.first)
			//	modifiedVertices.insert(vid);

			newInterval.first = iter->m_endTime + setupPath.size();//-1

			if(!m_mainDelivering.count(hvit->descriptor()) and !m_goalDelivering.count(hvit->descriptor())) {
				newInterval.first = newInterval.first + 1;
			}

			if(next != allocations.end()){
				auto departurePath = LowLevelGraphPath(_agent, next->m_endLocation,lowLevelVID,
															 			_constraints,
																		newInterval.first,//+1
																		next->m_startTime-1);

				newInterval.second = next->m_startTime - departurePath.size();
			}
			else {
				newInterval.second = MAX_INT;
			}

			if(newInterval != agentInterval.second) {
				modifiedVertices.insert(vid);
				auto cfg = m_availableIntervalGraph->GetVertex(vid);
				auto newVID = m_availableIntervalGraph->AddDuplicateVertex(cfg);
				newVertices[vid] = newVID;
				m_agentAvailableIntervalMap[newVID] = std::make_pair(_agent,newInterval);
			}
		}
	}


	for(auto vit = m_availableIntervalGraph->begin(); vit != m_availableIntervalGraph->end(); vit++) {
		//if(m_availableIntervalGraph->IsVertexInvalidated(vit->descriptor()))
		//	continue;

		//std::cout << "new vit: " << vit->descriptor() << std::endl;

		std::vector<std::pair<std::pair<size_t,size_t>,double>> sourceTargetWeight;

		for(auto eit = vit->begin(); eit != vit->end(); eit++) {
			auto source = eit->source();
			auto target = eit->target();
		//std::cout << eit->source() << std::endl;
		//std::cout << eit->target() << std::endl;
			bool modified = false;
			if(modifiedVertices.count(source)) {
				source = newVertices[source];
				modified = true;
			}
			if(modifiedVertices.count(target)) {
				target = newVertices[target];
				modified = true;
			}
			if(!modified)
				continue;
			sourceTargetWeight.push_back(std::make_pair(std::make_pair(source,target),eit->property().GetWeight()));
		}
		for(auto pair : sourceTargetWeight) {
			auto source = pair.first.first;
			auto target = pair.first.second;
			auto sourceCfg = m_availableIntervalGraph->GetVertex(source);
			auto targetCfg = m_availableIntervalGraph->GetVertex(target);

			auto sourceInterval = m_agentAvailableIntervalMap[source];
			auto targetInterval = m_agentAvailableIntervalMap[target];

			std::vector<size_t> path;

			if(sourceCfg.GetRobot()->GetCapability() == targetCfg.GetRobot()->GetCapability())
				path = LowLevelGraphPath(sourceCfg,targetCfg,
									_constraints,sourceInterval.second.first,
									targetInterval.second.first);
			else if(sourceCfg.GetRobot() == this->GetTaskPlan()->GetCoordinator()->GetRobot()) {

				auto iter = allocations.begin();

				auto cfg = m_availableIntervalGraph->GetVertex(target);
				cfg.SetRobot(this->GetTaskPlan()->GetCapabilityAgent(_agent->GetCapability())->GetRobot());

				auto lowLevelVID = this->GetCapabilityRoadmap(_agent)->GetVID(cfg);

				auto sTime = iter->m_endTime;
				if(sTime != 0) {
					sTime++;
				}

				path = LowLevelGraphPath(_agent, iter->m_endLocation,lowLevelVID,
															 			_constraints,
																		//iter->m_endTime,
																		sTime,
																		//agentInterval.second.first);
																		targetInterval.second.first);
			}
			if(path.size() == 0)
				continue;
      if(ValidIntervalEdge(source,target,path.size())) {
				DefaultWeight<Cfg> edge;
				if(!path.empty())
					edge.SetWeight(path.size());
				else
					edge.SetWeight(pair.second);
         m_availableIntervalGraph->AddEdge(source,target,edge);

			}
		}
	}

	//Make new vertices
	std::vector<size_t> newValid;
	std::vector<size_t> invalids;
	for(auto vid : modifiedVertices) {
		invalids.push_back(vid);
		newValid.push_back(newVertices[vid]);
		//m_availableIntervalGraph->SetVertexInvalidated(vid);
	}

	if(m_debug) {
		std::cout << "Post updating availability graph" << std::endl;
		PrintAvailabilityGraph();
	}
  return std::make_pair(invalids,newValid);
}
void
DiscreteIntervalGraph::
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
      size_t startTime = 0;
      size_t endTime = MAX_INT;

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
        size_t startTime = iter->GetEndTime()
                         + LowLevelGraphPathWeight(iter->GetEndLocation(),cfg);

				//if(startTime > 0 and iter->GetEndTime() == 0)
				if((startTime == 1 and iter->GetEndTime() == 0)
						or m_goalDelivering.count(vit->descriptor())
						or m_mainDelivering.count(vit->descriptor()))
					startTime--;

        auto next = iter;
        next++;

        size_t endTime;
        if(next == rat.end())
          endTime = MAX_INT;
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
	//	size_t vid = vit->descriptor();
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {
      for(auto s : m_intervalMap[eit->source()]) {
        for(auto t : m_intervalMap[eit->target()]) {
//          if(eit->target() == 15 or eit->source() == 8)
//            std::cout << "HERE" << vit->descriptor() << std::endl << "VID: " << vid << std::endl;
          if(ValidIntervalEdge(s,t,eit->property().GetWeight()))
            m_availableIntervalGraph->AddEdge(s,t,eit->property());
        }
      }
    }
  }
}

std::unordered_map<WholeTask*,std::vector<size_t>>&
DiscreteIntervalGraph::
GetTaskAigVIDs() {
  return m_aigTaskVIDs;
}
