#include "DiscreteMAD.h"

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/HandoffAgent.h"

#include "TMPLibrary/StateGraphs/DiscreteIntervalGraph.h"
#include "TMPLibrary/StateGraphs/MultiTaskGraph.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/WholeTask.h"

#include "Utilities/PMPLExceptions.h"

DiscreteMAD::
DiscreteMAD() : TMPBaseObject() {
  this->SetName("DiscreteMAD");
}

DiscreteMAD::
DiscreteMAD(XMLNode& _node) : TMPBaseObject (_node) {
  this->SetName("DiscreteMAD");
  m_sgLabel = _node.Read("sgLabel", true, "",
      "Label for the state graph used by DiscreteMAD.");
}

DiscreteMAD::
~DiscreteMAD(){};

std::vector<SubtaskPlan>
DiscreteMAD::
Run(WholeTask* _wholeTask, std::set<size_t> _validVIDs, ConstraintMap _constraints){
  if(!_wholeTask){
    throw RunTimeException(WHERE, "DiscreteMAD needs a wholeTask to solve.");
  }

  auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
  auto query = sg->AddTaskToGraph(_wholeTask,_validVIDs);

  auto start = query.first;
  auto goal = query.second;

	InitializeRobotUpdates(start);

  m_nodeAgentMap[start] = this->GetTaskPlan()->GetCoordinator();

  //Search backwards from goal to start
  SSSPPathWeightFunction<TaskGraph> weight;
  weight = [this,start,goal,_wholeTask,_constraints](typename AvailableIntervalGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    //return this->MAMTPathWeight(_ei,_sourceDistance,_targetDistance,goal,start);
    size_t sourceDistance;
		size_t targetDistance;
    if(_sourceDistance == std::numeric_limits<double>::infinity())
			sourceDistance = MAX_INT;
		else
			sourceDistance = size_t(_sourceDistance);
    if(_targetDistance == std::numeric_limits<double>::infinity())
			targetDistance = MAX_INT;
		else
			targetDistance = size_t(_targetDistance);
    return this->AvailableIntervalPathWeight(_ei,size_t(sourceDistance),size_t(targetDistance),
																						 start,goal,_wholeTask,_constraints);
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
    return {};
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

  auto subtaskPlans = ExtractTaskPlan(path,_wholeTask,sssp.distance,_constraints);
  sg->RemoveTaskFromGraph(_wholeTask);
  m_nodeAgentMap.clear();
  m_nodeRATCache.clear();
  m_parentMap.clear();
  m_incomingEdgeMap.clear();
  m_usedAgents.clear();
	m_robotUpdates.clear();
  return subtaskPlans;
}

std::vector<SubtaskPlan>
DiscreteMAD::
ExtractTaskPlan(const std::vector<size_t>& _path, WholeTask* _wholeTask,
    std::unordered_map<size_t,double> _distance, ConstraintMap _constraints){
return {};
/*
	std::vector<SubtaskPlan> taskPlan;

  auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
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
			if(_distance[vid] == MAX_INT)
				throw RunTimeException(WHERE,"Invalid Plan.");
    }
  }

	Robot* previousRobot = nullptr;
	size_t first = 0;
	size_t last = 0;
	for(size_t i = 0; i < _path.size(); i++) {
		auto vid = _path[i];
		Robot* robot;

		if(i == 0 or i == _path.size()-1) {
			robot = this->GetTaskPlan()->GetCoordinator()->GetRobot();
		}
		else {
			robot = g->GetVertex(vid).GetRobot();
		}
		//Check if continueing the same subtask
		if(robot == previousRobot) {
			last = vid;
			continue;
		}
		else if(previousRobot and previousRobot != this->GetTaskPlan()->GetCoordinator()->GetRobot()){
			SubtaskPlan plan = CreateSubtaskPlan(static_cast<HandoffAgent*>(previousRobot->GetAgent()),
																					 first, last,_distance[first], _distance[vid],//_distance[last],
																					 _constraints);
			taskPlan.push_back(plan);
		}
		first = vid;
		last = vid;
		previousRobot = robot;
	}

	return taskPlan;*/
}

std::shared_ptr<MPTask>
DiscreteMAD::
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
DiscreteMAD::
AvailableIntervalPathWeight(typename AvailableIntervalGraph::adj_edge_iterator& _ei,
    size_t _sourceDistance, size_t _targetDistance, size_t _start, size_t _goal,
		WholeTask* _task, ConstraintMap _constraints) {

  size_t edgeWeight  = size_t(_ei->property().GetWeight() + 0.5);
  size_t source = _ei->source();
  size_t target = _ei->target();

  size_t newDistance = std::numeric_limits<size_t>::infinity();

  auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());

  //if(sg->GetAvailableIntervalGraph()->IsVertexInvalidated(target)){
  //  return std::numeric_limits<double>::infinity();
	//}

  auto cfg = sg->GetAvailableIntervalGraph()->GetVertex(target);
  auto sourceCfg = sg->GetAvailableIntervalGraph()->GetVertex(source);

	/*if(target == _goal) {
		target = MAX_INT;
		edgeWeight = MAX_INT;
	}	*/
  auto transition = sg->ValidTransition(source,target,edgeWeight,_sourceDistance,
                    m_robotUpdates[source][cfg.GetRobot()->GetAgent()], _constraints);

  if(transition >= 0){
		//TODO::Check if this violates a positive constraint
		size_t end = _sourceDistance + transition;
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
						//newDistance = std::numeric_limits<size_t>::infinity();
						return std::numeric_limits<double>::infinity();
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

  return double(newDistance);
}

SubtaskPlan
DiscreteMAD::
CreateSubtaskPlan(HandoffAgent* _agent, size_t _start, size_t _end, size_t _startTime, size_t _endTime,
									ConstraintMap _constraints) {

	if(_endTime == MAX_INT or _startTime == MAX_INT)
		throw RunTimeException(WHERE,"Invalid Plan.");

	SubtaskPlan plan;
	plan.m_agent = _agent;
	plan.m_taskStartVID = _start;
	plan.m_taskEndVID = _end;
	plan.m_subtaskStartTime = _startTime;

  auto sg = static_cast<DiscreteIntervalGraph*>(this->GetStateGraph(m_sgLabel).get());
	auto roadmap = sg->GetCapabilityRoadmap(_agent);

	auto allocations =	this->GetTaskPlan()->GetAgentAllocations(_agent);

	auto iter = allocations.begin();
	while(iter != allocations.end()) {
		if(iter->m_endTime > _startTime)
			break;
		iter++;
	}
	if(iter == allocations.begin())
		throw RunTimeException(WHERE,
							"This should never happen because the first end time should be 0"
							" from the initial positioning of the robots.");
	iter--;

	//Setup path

	auto setupStart = iter->m_endLocation;
	auto setupBegin = iter->m_endTime;
	if(setupBegin > 0)
		setupBegin++;

	Cfg setupCfg = roadmap->GetVertex(setupStart);
	setupCfg.SetRobot(_agent->GetRobot());

	Cfg startCfg = sg->GetAvailableIntervalGraph()->GetVertex(_start);
	startCfg.SetRobot(_agent->GetRobot());

	Cfg goalCfg = sg->GetAvailableIntervalGraph()->GetVertex(_end);
	goalCfg.SetRobot(_agent->GetRobot());

	auto setupPath = sg->LowLevelGraphPath(setupCfg, startCfg, _constraints, setupBegin, _startTime-1);
	if(setupPath.size() + setupBegin > _startTime+1)
		throw RunTimeException(WHERE,"This should never happen due to the available interval concept of "
																 "the availability graph.");
		//may be off by one so double check that it doesn't need to be path.size() - 1
	if(setupPath.size() == 1)
		plan.m_setupPath = {};
	else
		plan.m_setupPath = setupPath;
	plan.m_setupStartTime = setupBegin;

	//Execution Path
	auto executionPath = sg->LowLevelGraphPath(startCfg, goalCfg, _constraints, _startTime, _endTime);
	if(executionPath.size() + _startTime > _endTime+1 or executionPath.empty())
		throw RunTimeException(WHERE, "The execution path violates the task path found in the "
																	"availability graph search.");

	plan.m_subtaskPath = executionPath;

	return plan;
}

void
DiscreteMAD::
InitializeRobotUpdates(size_t _start) {

  for(auto agent : this->GetTaskPlan()->GetTeam()) {
    //Cfg cfg(agent->GetRobot());
		auto occInt = this->GetTaskPlan()->GetRobotAvailability(agent).front();
		auto cfg = occInt.GetEndLocation();

		int x = int(cfg[0] + .5);
		int y = int(cfg[1] + .5);

		cfg.SetData({double(x),double(y),0});

		cfg.SetRobot(agent->GetRobot());
    m_robotUpdates[_start][agent] = std::make_pair(0,cfg);
  }
}
