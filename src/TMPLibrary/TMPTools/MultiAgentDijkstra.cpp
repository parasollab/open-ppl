#include "MultiAgentDijkstra.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/PMPLExceptions.h"

MultiAgentDijkstra::
MultiAgentDijkstra() : TMPBaseObject() {
  this->SetName("MultiAgentDijkstra");
}

MultiAgentDijkstra::
MultiAgentDijkstra(XMLNode& _node) : TMPBaseObject (_node) {
  this->SetName("MultiAgentDijkstra");
  m_sgLabel = _node.Read("sgLabel", true, "",
      "Label for the state graph used by MultiAgentDijkstra.");
}

MultiAgentDijkstra::
~MultiAgentDijkstra(){};

bool
MultiAgentDijkstra::
Run(WholeTask* _wholeTask, std::shared_ptr<TaskPlan> _plan, std::set<size_t> _validVIDs){
  if(!_wholeTask){
    throw RunTimeException(WHERE, "MultiAgentDijkstra needs a wholeTask to solve.");
  }
  std::shared_ptr<TaskPlan> savedPlan = nullptr;
  if(_plan){
    savedPlan = this->GetTaskPlan();
    this->GetTMPLibrary()->SetTaskPlan(_plan);
  }

  auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto query = sg->AddTaskToGraph(_wholeTask,_validVIDs);

  m_overage = 0;

  auto start = query.first;
  auto goal = query.second;

  for(auto agent : this->GetTaskPlan()->GetTeam()) {
    Cfg cfg(agent->GetRobot());
    m_robotUpdates[start][agent] = std::make_pair(0,cfg);
  }

  m_nodeAgentMap[start] = this->GetTaskPlan()->GetCoordinator();

  //Search backwards from goal to start
  SSSPPathWeightFunction<TaskGraph> weight;
  weight = [this,start,goal,_wholeTask](typename AvailableIntervalGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    //return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance,goal,start);
    return this->AvailableIntervalPathWeight(_ei,_sourceDistance,_targetDistance,start,goal,_wholeTask);
  };

  //TODO::Actually call dijkstras

  SSSPTerminationCriterion<TaskGraph> termination(
      [goal](typename TaskGraph::vertex_iterator& _vi,
        const SSSPOutput<TaskGraph>& _sssp) {
      return goal == _vi->descriptor() ? SSSPTermination::EndSearch
      : SSSPTermination::Continue;
      }
      );
  //auto g = this->GetStateGraph(m_sgLabel)->GetGraph();
  auto g = sg->GetAvailableIntervalGraph();
  const SSSPOutput<TaskGraph> sssp = DijkstraSSSP(g.get(), {start}, weight, termination);

  const size_t last = sssp.ordering.back();
  if(goal != last){
    return false;
    //return new TaskPlan();
  }

  //Extract path
  std::vector<size_t> path;
  path.push_back(last);

  size_t current = last;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != start);
  std::reverse(path.begin(), path.end());

  ExtractTaskPlan(path,_wholeTask,sssp.distance);
  sg->RemoveTaskFromGraph(_wholeTask);
  m_nodeAgentMap.clear();
  m_nodeRATCache.clear();
  m_parentMap.clear();
  m_incomingEdgeMap.clear();
  m_usedAgents.clear();
  if(savedPlan)
    this->GetTMPLibrary()->SetTaskPlan(savedPlan);
  return true;
}

void
MultiAgentDijkstra::
ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask,
    std::unordered_map<size_t,double> _distance){
  auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto g = sg->GetAvailableIntervalGraph();
  if(m_debug){
  	sg->PrintAvailabilityGraph();
    std::cout << "Print path." << std::endl;
    for(auto vid : _path){
      std::string label;
      if(vid == _path.front() or vid == _path.back()){
        label = "Virtual";
      }
      else {
        label = g->GetVertex(vid).GetRobot()->GetLabel();
				if(_distance[vid] == MAX_DBL)
					std::cout << "This is a problem" << std::endl;
      }
      std::cout << vid << " : " << _distance[vid] << " : " << label <<std::endl;
    }
  }
  //auto g = this->GetStateGraph(m_sgLabel)->GetGraph();
  //TaskPlan* taskPlan = new TaskPlan();
  auto taskPlan = this->GetTaskPlan();
  //taskPlan->AddWholeTask(_wholeTask);
  Agent* currentAgent = nullptr;
  std::shared_ptr<MPTask> previousTask(nullptr);
  //bool newTask = true;
  size_t first;
  size_t last;
  //double currentTime = 0;
  double startTime = 0;
  //TODO::Think about if we want to back the first subtask up to the start of
  //the second one so the robot is available up until the wait time is 0.
  //Also, maybe look at reducing wait time across everything as a metric later.
  double endTime = sg->GetInterval(_path[1]).second.first;

	taskPlan->GetTIM()[_wholeTask] = {};

  for(size_t i = 1; i < _path.size(); i++) {
    auto interval = sg->GetInterval(_path[i]);
    if(currentAgent != interval.first) {
      if(currentAgent) {//skips first index
        auto start = g->GetVertex(first);
        auto goal = g->GetVertex(last);
				//Account for waiting time unless for the final virtual node
        if(i == _path.size()-1)
					endTime = _distance[last];
				else
					endTime = _distance[_path[i]];
	
				if(endTime == 0)
					std::cout << "THIS IS A PROBLEM" << std::endl;

        auto task = CreateMPTask(currentAgent->GetRobot(), start, goal,_wholeTask);
        taskPlan->AddSubtask(static_cast<HandoffAgent*>(currentAgent),task,_wholeTask);

        OccupiedInterval interval(static_cast<HandoffAgent*>(currentAgent),
                                  start,goal,startTime,endTime);
        interval.SetTask(task);

        if(m_debug){
          std::cout << interval.Print() << std::endl;
        }

        taskPlan->UpdateTIM(_wholeTask,interval);
        taskPlan->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),interval);

        if(previousTask){
          taskPlan->AddDependency(task,previousTask);
        }

        previousTask = task;
      }
      first = _path[i];
      currentAgent = interval.first;
      startTime = endTime;
    }
    else {
      last = _path[i];
    }
  }

  this->GetTaskPlan()->SetPlanCost(_wholeTask,_distance[_path[1]],_distance[_path[_path.size()-2]]);
/*
  for(size_t i = 2; i < _path.size(); i++){
    auto vid = _path[i];
    if(currentAgent and !m_nodeAgentMap[vid]){
      auto start = g->GetVertex(first);
      auto goal = g->GetVertex(last);
      start.SetRobot(goal.GetRobot());

      auto task = CreateMPTask(currentAgent->GetRobot(), start, goal,_wholeTask);
      taskPlan->AddSubtask(static_cast<HandoffAgent*>(currentAgent),task,_wholeTask);

      auto endTime = _distance[last];
      auto startTime = endTime - m_incomingEdgeMap[first];

      if(i < _path.size()-2)//buffer to account for wait time to interact
        endTime = _distance[_path[i+2]] - m_incomingEdgeMap[_path[i+1]];

      OccupiedInterval interval(static_cast<HandoffAgent*>(currentAgent),start,goal,startTime,endTime);
      interval.SetTask(task);
      if(m_debug){
        std::cout << interval.Print() << std::endl;
      }
      taskPlan->UpdateTIM(_wholeTask,interval);
      taskPlan->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),interval);
      if(previousTask){
        taskPlan->AddDependency(task,previousTask);
      }
      previousTask = task;
      if(vid == _path.back()){
        vid = _path[i-1];
      }

      currentAgent = nullptr;
      continue;
    }
    else if(!currentAgent){
      currentAgent = m_nodeAgentMap[vid];
      first = vid;
    }
    else if(currentAgent == m_nodeAgentMap[vid]){
      last = vid;
    }
  }
*/

  /*for(size_t i = 0; i < _path.size(); i++){
    auto vid = _path[i];
    if(vid == _path.front()){
      currentTime += m_extractionCostMap[vid];
      startTime = currentTime;
      continue;
    }
    if(!currentAgent){ //should only be the case for the first node
      currentAgent = m_nodeAgentMap[vid];
      currentTime += m_extractionCostMap[vid];
      newTask = false;
      first = vid;
    }
    //need to keep all virtual nodes virtual or maybe not...
    // The virtual node is really just a copy of the real cfg
    // This is a virtual node
    else if((m_nodeAgentMap[vid] != currentAgent and m_nodeAgentMap[vid])or vid == _path.back()){
      if(m_debug){
        std::cout << "New task: " << first << "->" << last << std::endl;
      }
      auto start = g->GetVertex(first);
      auto goal = g->GetVertex(last);
      auto task = CreateMPTask(currentAgent->GetRobot(), start, goal,_wholeTask);
      taskPlan->AddSubtask(static_cast<HandoffAgent*>(currentAgent),task,_wholeTask);

      //make interval

      OccupiedInterval interval(static_cast<HandoffAgent*>(currentAgent),start,goal,startTime,currentTime);
      interval.SetTask(task);
      if(m_debug){
        std::cout << interval.Print() << std::endl;
      }
      taskPlan->UpdateTIM(_wholeTask,interval);
      taskPlan->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),interval);
      if(previousTask){
        taskPlan->AddDependency(task,previousTask);
      }
      previousTask = task;
      if(vid == _path.back()){
        vid = _path[i-1];
      }
      goal.SetRobot(currentAgent->GetRobot());
      //TODO::Update to occupied interval logic
      //auto avail = std::pair<Cfg,double>(goal,_distance[vid]);//May need to set robot pointer for goal
      //this->GetTaskPlan()->UpdateRAT(static_cast<HandoffAgent*>(currentAgent),avail);
      currentAgent = m_nodeAgentMap[vid];
      currentTime = std::max(currentTime,m_extractionCostMap[vid]);
      startTime = currentTime;
      newTask = true;
    }
    else{
      if(newTask){
        first = vid;
        newTask = false;
      }
      last = vid;
      currentTime += m_extractionCostMap[vid];
    }
  }*/
  //Update RAT
  /*for(auto avail : m_nodeRATCache[_path.back()]){
    for(auto interval : avail.second){
      this->GetTaskPlan()->UpdateRAT(static_cast<HandoffAgent*>(avail.first),interval);
    }
  }*/
  //TODO::Update TIM
//  this->GetTaskPlan()->SetPlanCost(_wholeTask,_distance[_path.front()]);
}

std::shared_ptr<MPTask>
MultiAgentDijkstra::
CreateMPTask(Robot* _robot, Cfg _start, Cfg _goal, WholeTask* _wholeTask){
  std::shared_ptr<MPTask> task(new MPTask(_robot));
  _start.SetRobot(_robot);
  _goal.SetRobot(_robot);

  if(!_goal.GetRobot()->IsManipulator()){
    auto radius = (_robot->GetMultiBody()->GetBoundingSphereRadius());

    std::unique_ptr<CSpaceBoundingBox> boundingBox(
        new CSpaceBoundingBox({_start.GetPosition()[0],_start.GetPosition()[1],0}));

    boundingBox->SetRange(0,
        (_start.GetPosition()[0]-radius),
        (_start.GetPosition()[0]+radius));
    boundingBox->SetRange(1,
        (_start.GetPosition()[1]-radius),
        (_start.GetPosition()[1]+radius));
    boundingBox->SetRange(2,-1,1);

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_start.GetRobot(), std::move(boundingBox)));
    std::unique_ptr<CSpaceBoundingBox> boundingBox2(
        new CSpaceBoundingBox({_goal.GetPosition()[0],_goal.GetPosition()[1],0}));

    boundingBox2->SetRange(0,
        (_goal.GetPosition()[0]-radius/2),
        (_goal.GetPosition()[0]+radius/2));
    boundingBox2->SetRange(1,
        (_goal.GetPosition()[1]-radius/2),
        (_goal.GetPosition()[1]+radius/2));
    boundingBox2->SetRange(2,-1,1);

    auto goalConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(_goal.GetRobot(), std::move(boundingBox2)));

    task->SetStartConstraint(std::move(startConstraint));
    task->ClearGoalConstraints();
    task->AddGoalConstraint(std::move(goalConstraint));
  }
  else{
    throw RunTimeException(WHERE, "Manipulator code not yet written for MAMTP.");
  }

  auto& map = _wholeTask->m_interactionPoints;
  std::vector<Cfg> startPath = {};
  std::vector<Cfg> goalPath = {};
  for (auto& interactionPoint : map){
    if (*interactionPoint.first == _start){
      startPath = *interactionPoint.second;
    }
    else if (*interactionPoint.first == _goal){
      goalPath = *interactionPoint.second;
    }
  }

  _wholeTask->m_interactionPathsDelivering[task] = goalPath;
  _wholeTask->m_interactionPathsReceiving[task] = startPath;

  return task;
}


double
MultiAgentDijkstra::
MAMTPathWeight(typename TaskGraph::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance, size_t start, size_t _goal) {

  const double edgeWeight  = _ei->property().GetWeight();
  double readyTime = 0;
  Agent* newAgent = nullptr;
  size_t source = _ei->source();
  size_t target = _ei->target();

  //Check if edge is virtual and if so find the next robot and keep track of current robot in a map in this class
  bool virt = (edgeWeight == -1);
  size_t parent = -1;
  double incomingEdge = -1;
  if(virt){
    if(m_parentMap.empty())//virtual goal node in backwards search
      return 0;
    // TODO::figure out if these checks are actually needed-may be the case that these always exist given
    // its a virtual node
    //if(std::find(m_parentMap.begin(),m_parentMap.end(),source) != m_parentMap.end()) {
    parent = m_parentMap[source];
    incomingEdge = m_incomingEdgeMap[source];
    //}
  }
  if(virt){// and target !=_goal){
    auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());
    readyTime = sg->RobotSelection(source,target,&newAgent,
        m_nodeRATCache[source], parent, incomingEdge, m_usedAgents[source]);
  }
  else{
    newAgent = m_nodeAgentMap[source];
  }

  // Compute the new 'distance', which is the number of timesteps at which
  // the robot would reach the target node.
  double newDistance = MAX_DBL;
  if(virt){
    //This is the old forward implementation
    //Check if the robot will need extra time to reach the location
    //if(readyTime > _sourceDistance){
    //	newDistance = readyTime + incomingEdge;
    //}
    //Or if it can be there when the previous robot reaches the interaction point
    //else{
    //	newDistance = _sourceDistance + incomingEdge;
    //}

		if(readyTime < _sourceDistance - incomingEdge)
			newDistance = _sourceDistance;
		else
			newDistance = readyTime + incomingEdge;

    // receiving < delivering
    // The agent on the delivering end of the previously evaluated interaction can arrive before the
    // just discovered agent can deliver the task to the IT.
    // We over compensated the readyTime of the previous agent and need to subtract it.
    // This does not ruin the dijkstra search because only robots of the specified type can use the edge,
    // and this is still the first robot that can get there.
    /*double virtualWeight = -1*m_overage;
    if(_sourceDistance < readyTime) {
      double oldReadyTime = 0;
      // If the subtask is dirent from start to goal, there is no oldReadyTime to account for.
      auto it = m_incomingEdgeMap.find(parent);
      if(it != m_incomingEdgeMap.end())	{
        oldReadyTime = m_incomingEdgeMap[parent];
      }
      //newDistance = _sourceDistance - oldReadyTime + readyTime;
      m_overage -= oldReadyTime;
    }
    // receiving > delivering
    // The agent on the receiving end of the previously evaluated interaction arrives after the
    // just discovered agent delivers the task.
    // We have already accounted for the readyTime of the just discovered agent in the readyTime of the
    // previous agent.
    else {
      //newDistance = _sourceDistance - readyTime;
      m_overage -= (readyTime + m_incomingEdgeMap[source]);
    }*/
    //m_overage = std::max(m_overage, readyTime);
    //virtualWeight += m_overage;
    //newDistance = _sourceDistance + virtualWeight;
  }
  else{
    newDistance = _sourceDistance + edgeWeight;
  }

  // If this edge is better than the previous we update the robot at the node
  if(newDistance < _targetDistance) {
    //m_usedAgents[target] = m_usedAgents[source];
		m_nodeRATCache[target] = m_nodeRATCache[source];
    if(virt){
      //m_usedAgents[target].insert(static_cast<HandoffAgent*>(newAgent));
      m_nodeAgentMap[target] = newAgent;
      m_nodeAgentMap[source] = newAgent;
      //m_nodeAgentMap[parent] = newAgent;
      //if(m_parentMap[parent] == start)//checks the initial motion
        //m_nodeAgentMap[start] = newAgent;
      /*
      m_nodeRATCache[target] = m_nodeRATCache[source];
      Cfg endLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(parent);
      endLocation.SetRobot(newAgent->GetRobot());
      double endTime = readyTime + incomingEdge;
      Cfg startLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(source);
      startLocation.SetRobot(newAgent->GetRobot());

      auto interval = new OccupiedInterval(static_cast<HandoffAgent*>(newAgent), startLocation,
          endLocation, readyTime, endTime);

      m_nodeRATCache[target][newAgent].push_back(interval);*/

      Cfg startLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(source);
      startLocation.SetRobot(newAgent->GetRobot());
      Cfg endLocation = this->GetStateGraph(m_sgLabel)->GetGraph()->GetVertex(target);
      endLocation.SetRobot(newAgent->GetRobot());
			double startTime = readyTime;
			double endTime = readyTime + incomingEdge;

			OccupiedInterval interval(static_cast<HandoffAgent*>(newAgent), startLocation,
					endLocation, startTime, endTime);

			m_nodeRATCache[target][newAgent].push_back(interval);
      m_extractionCostMap[target] = readyTime;
    }
    else {
			m_nodeAgentMap[target] = nullptr;
      m_extractionCostMap[target] = edgeWeight;
    }
    m_parentMap[target] = source;
    m_incomingEdgeMap[target] = newDistance - _sourceDistance;

  }
  if(!m_nodeAgentMap[target])
    std::cout << "problem here" << std::endl;
  return newDistance;
}


double
MultiAgentDijkstra::
AvailableIntervalPathWeight(typename AvailableIntervalGraph::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance, size_t _start, size_t _goal,
		WholeTask* _task) {

  const double edgeWeight  = _ei->property().GetWeight();
  size_t source = _ei->source();
  size_t target = _ei->target();

  double newDistance = std::numeric_limits<double>::infinity();

  auto sg = static_cast<MultiTaskGraph*>(this->GetStateGraph(m_sgLabel).get());

  if(sg->GetAvailableIntervalGraph()->IsVertexInvalidated(target)){
					if(source == 1517 and target == 45)
									std::cout << "THIS SHOULD BE WORKING" << std::endl;

    return newDistance;
	}

  auto cfg = sg->GetAvailableIntervalGraph()->GetVertex(target);
  auto sourceCfg = sg->GetAvailableIntervalGraph()->GetVertex(source);

  auto transition = sg->ValidTransition(source,target,edgeWeight,_sourceDistance,
                    m_robotUpdates[source][cfg.GetRobot()->GetAgent()]);

  if(transition >= 0){
		//TODO::Check if this violates a positive constraint
		double end = _sourceDistance + transition;
  	newDistance = end;
		if(cfg.GetRobot() == sourceCfg.GetRobot()){
			/*for(auto constraint : this->GetTaskPlan()->GetPositiveTaskConstraints(_task)) {
				if((_sourceDistance >= constraint.GetStartTime() and _sourceDistance < constraint.GetStartTime())){
					//or (end <= constraint.GetEndTime() and end > constraint.GetStartTime())) {
					if(cfg.GetRobot()->GetAgent() != constraint.GetAgent() or
						 cfg != constraint.GetEndLocation() or 
							sg->GetAvailableIntervalGraph()->GetVertex(source) != constraint.GetStartLocation()) {

						newDistance = std::numeric_limits<double>::infinity();	
					}
					break;
				}
			}*/
			for(auto instant : this->GetTaskPlan()->GetPositiveInstantTaskConstraints(_task)) {
				if(instant.second >= _sourceDistance and instant.second <= _sourceDistance + transition){
					if(cfg.GetRobot() != instant.first->GetRobot())
						newDistance = std::numeric_limits<double>::infinity();
					break;
				}
			}
		}
		/*
		for(auto inst : this->GetTaskPlan()->GetPositiveInstantTaskConstraints(_task)) {
			if(inst.second >= _sourceDistance and inst.second <= _sourceDistance+transition and
				 inst.first->GetRobot() != cfg.GetRobot() and inst.first->GetRobot() != sourceCfg.GetRobot())
				newDistance = std::numeric_limits<double>::infinity();
		}
		*/
  	//newDistance = end;
	}	

  if(newDistance < _targetDistance) {
    m_robotUpdates[target] = m_robotUpdates[source];
    m_robotUpdates[target][cfg.GetRobot()->GetAgent()] = std::make_pair(newDistance,cfg);
  }
	if(source == 1517 and target == 45 and newDistance == std::numeric_limits<double>::infinity())
		std::cout << "THIS SHOULD BE WORKING" << std::endl;

  return newDistance;
}
