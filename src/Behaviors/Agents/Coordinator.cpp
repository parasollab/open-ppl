#include "Coordinator.h"

#include <limits>
#include <queue>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

#include "MPLibrary/MPTools/TRPTool.h"

#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/InteractionInformation.h"
#include "MPProblem/Robot/Robot.h"

#include "Simulator/Simulation.h"
#include "Simulator/BulletModel.h"

#include "Traits/CfgTraits.h"

#include "TMPLibrary/Actions/Action.h"
#include "TMPLibrary/TaskPlan.h"
#include "TMPLibrary/TMPStrategies/ITMethod.h"
#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

#include "sandbox/gui/main_window.h"

/*------------------------------ Construction --------------------------------*/

Coordinator::
Coordinator(Robot* const _r) : Agent(_r) {
}


Coordinator::
Coordinator(Robot* const _r, XMLNode& _node) : Agent(_r, _node) {

  // Parse the labels of the group members.
  for(auto& child : _node) {
    // Load the environment file used to create handoff templates
    /*if(child.Name() == "HandoffEnvironment") {
      // Ignore this node if we already have an environment.
      if(!m_handoffEnvironment)
        m_handoffEnvironment = std::unique_ptr<Environment>(new Environment(child));
    }*/
    /*else if(child.Name() == "PlacementMethod") {
      std::cout << "Reading Placement Method" << std::endl;
      AddPlacementMethod(PlacementMethod::Factory(m_robot->GetMPProblem(), child));
    }*/
    //else {
    if(child.Name() == "Member") {
      // Parse the robot label.
      const std::string memberLabel = child.Read("label", true, "",
          "The label of the member robot.");
      m_memberLabels.push_back(memberLabel);
    }
  }

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");

  //m_tmp = _node.Read("tmp", false, false, "Does the coordinator use a tmp method?");

  //m_it = _node.Read("it", false, true, "Generate the Capability and Combined Roadmaps.");

  //m_connectionThreshold = _node.Read("connectionThreshold",true,1.2, 0., 1000.,
  //    "Acceptable variabliltiy in IT paths.");

  // This is a coordinator agent, which does not make sense without some group
  // members to coordinate. Throw an exception if it has no members.
  if(m_memberLabels.empty())
    throw ParseException(_node.Where(), "Coordinator requires at "
        "least one member robot.");
}


Coordinator::
~Coordinator() {
  Uninitialize();
}


std::unique_ptr<Agent>
Coordinator::
Clone(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not yet implemented.");
  return {nullptr};
}
/*------------------------------ Agent Interface -----------------------------*/

void
Coordinator::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_tmpLibrary = new TMPLibrary(xmlFile);
  m_library = m_tmpLibrary->GetMPLibrary();
	m_solution = new MPSolution(m_robot);
  //m_library->SetMPSolution(m_solution);

  // Set up the group members.
  int priority = 1;
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);

    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    Agent* memberAgent = member->GetAgent();

    HandoffAgent* a = dynamic_cast<HandoffAgent*>(
        memberAgent);
    if(!a)
      throw RunTimeException(WHERE, "Incompatible agent type specified for "
          "group member '" + memberLabel + "'.");
    m_memberAgents.push_back(a);

    // Set the initial priority.
    SetPriority(memberAgent, priority++);
  }

  // Initialize the version map.
  for(Agent* agent : m_memberAgents){
    std::unordered_map<PlanningAgent*, size_t> otherMap;
    for(Agent* otherAgent : m_memberAgents){
      if(agent != otherAgent){
        PlanningAgent* planningAgent = dynamic_cast<PlanningAgent*>(otherAgent);
        if(!planningAgent)
          throw RunTimeException(WHERE, "Incompatible agent type specified for "
              "group member '" + planningAgent->GetRobot()->GetLabel() + "'.");
        otherMap[planningAgent] = 0;
      }
    }
    m_versionMap[agent] = otherMap;
  }


  for(auto agent : m_memberAgents){
    std::cout << agent->GetRobot()->GetLabel() << std::endl;
  }

  InitializeAgents();
  if(m_debug){
    std::cout << "Done Initializing Agents" << std::endl;
  }

	// if networked, request plan from server
	if(m_communicator.IsConnectedToMaster()) {
		std::string query  ="send me a task plan";
		std::cout << m_communicator.Query("ppl",query) << std::endl;
	}
	//else use tmplibrary to get task assignments
	else {
		TaskPlan* taskPlan = new TaskPlan();
		m_tmpLibrary->Solve(problem, problem->GetTasks(m_robot), taskPlan, this, m_memberAgents);
  	DistributeTaskPlan(taskPlan);
	}

  if(m_debug){
    std::cout << "OTUPUTTING AGENT TASK ASSIGNMENTS" << std::endl;
    for(auto agent : m_memberAgents){
      std::cout << agent->GetRobot()->GetLabel() << std::endl;
      auto list = agent->GetQueuedSubtasks();
      for(auto task : list){
        std::cout << task << std::endl;
      }
    }

    std::cout << "Looking at whole tasks now" << std::endl;
    for(auto& wholeTask : m_tmpLibrary->GetTaskPlan()->GetWholeTasks()){
      if(m_debug){
        std::cout << "New whole task" << std::endl;
      }
      for(auto task : wholeTask->m_subtasks){
        std::cout << task << std::endl;
      }
    }
  }

  for(auto agent : m_memberAgents){
    agent->GetRobot()->SetVirtual(false);
  }

  Simulation::Get()->PrintStatFile();
  m_clock.start();
}


void
Coordinator::
Step(const double _dt) {
  Initialize();

  if(this->m_debug)
    std::cout << "___________________________________________________________"
              << std::endl;
  for(auto agent : m_memberAgents)  {
    if(this->m_debug)
      std::cout << agent->GetRobot()->GetLabel()
                << " has plan: "
                << agent->HasPlan()
                << std::endl
                << "Has task: "
                << agent->GetTask().get()
                << std::endl;
  }

  if(!m_robot->IsManipulator())
    ArbitrateCollision();

  for(auto agent : m_memberAgents){
    agent->Step(_dt);
  }

  CheckFinished();

  m_currentTime += m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
}


void
Coordinator::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  delete m_solution;
  delete m_library;
	delete m_tmpLibrary;  
//delete m_megaRoadmap;

  m_solution = nullptr;
  m_library  = nullptr;
  //m_megaRoadmap = nullptr;

  for(auto id : m_simulatorGraphIDs){
    Simulation::Get()->RemoveRoadmap(id);
  }

  //for(auto& graph : m_capabilityRoadmaps){
  //  delete graph.second;
  //}
}

/*-------------------------- Coordinator Interface ---------------------------*/

void
Coordinator::
ArbitrateCollision() {
  std::vector<std::pair<Agent*, std::vector<Agent*>>> needReplan;
  for(auto agent : m_memberAgents) {
    HandoffAgent* childAgent =
      static_cast<HandoffAgent*>(agent);
    if(childAgent->IsPlanning())
      continue;
    const double distanceThreshold = 4. *
      childAgent->GetRobot()->GetMultiBody()->GetBoundingSphereRadius();
    auto group = childAgent->ProximityCheck(distanceThreshold);
    if(!group.empty() and !ValidateVersionMap(childAgent, group)){
       needReplan.push_back(std::make_pair(childAgent, group));

       if(this->m_debug)
         std::cout << childAgent->GetRobot()->GetLabel() << " is in collision"
                   << std::endl;
       childAgent->ClearPlan();
       childAgent->ResetStartConstraint();
    }
  }
  for(auto pair : needReplan){
    HandoffAgent* groupAgent = static_cast<HandoffAgent*>(pair.first);
    vector<Agent*> group = pair.second;
    if(IsHighestPriority(groupAgent, group)){
      if(this->m_debug)
        std::cout << "Updating Version Map" << std::endl;
      UpdateVersionMap(groupAgent, group);
    }
    else{
      Cfg currentPosition = groupAgent->GetRobot()->GetSimulationModel()->GetState();
      groupAgent->SetPlan({currentPosition});
      groupAgent->PauseAgent(5);
    }
  }
}

void
Coordinator::
CheckFinished() {
  //if(!m_unassignedTasks.empty())
  //  return;
  for(auto& wholeTask : m_tmpLibrary->GetTaskPlan()->GetWholeTasks()){
    if(!wholeTask->m_task->GetStatus().is_complete())
      return;
  }
  for(auto agent : m_memberAgents){
    if(!agent->GetQueuedSubtasks().empty())
      return;
		if(agent->GetTask())
			return;
  }
  for(auto agent : m_memberAgents){
    agent->ClearVisualGraph();
  }
  if(m_debug){
    for(auto& wholeTask : m_tmpLibrary->GetTaskPlan()->GetWholeTasks()){
      std::cout << wholeTask << std::endl;
    }
  }
  m_clock.stop();
  Simulation::GetStatClass()->SetStat("SimulationTime", m_clock.elapsed());
  //no incomplete tasks and no agent still performing a task
  //std::string statName = "STAT-"+m_robot->GetMPProblem()->GetHandoffTemplates()[0]->GetLabel();
  Simulation::Get()->PrintStatFile();
  exit(0);

}

/*--------------------------- Member Management ------------------------------*/

void
Coordinator::
SetPriority(Agent* const _a, const size_t _priority) {
  if(this->m_debug)
    std::cout << "Setting priority of: " << _priority << std::endl
            << "For: " << _a->GetRobot()->GetLabel() << std::endl;
  m_memberPriorities[_a] = _priority;
}


size_t
Coordinator::
GetPriority(Agent* const _a) {
  return m_memberPriorities[_a];
}

bool
Coordinator::
IsHighestPriority(Agent* const _a, const vector<Agent*>& _group){
  size_t maxPriority = 0;
  for(auto a : _group){
    HandoffAgent* agent = static_cast<HandoffAgent*>(a);
    size_t currentPriority = agent->GetPriority();
    if(currentPriority >= maxPriority)
      maxPriority = currentPriority;
  }
  //used to break ties
  HandoffAgent* _agent = static_cast<HandoffAgent*>(_a);
  if(_agent->GetPriority() == maxPriority)
    _agent->SetPriority(maxPriority+1);
  return _agent->GetPriority() > maxPriority;
}

void
Coordinator::
DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where) {
  // Create a task to send the member to the desired location. Use the
  // coordinator robot because this is shared-roadmap planning.
  std::shared_ptr<MPTask> task(new MPTask(m_robot));
  std::unique_ptr<BoundaryConstraint> destination(
      new BoundaryConstraint(m_robot, std::move(_where))
  );


  task->AddGoalConstraint(std::move(destination));
  if(this->m_debug) {
    std::cout << "SENDING " << _member->GetRobot()->GetLabel()
              << " TO:" << std::endl;
    for(const auto& constraint : task->GetGoalConstraints())
      std::cout << "\t" << *(constraint->GetBoundary()) << std::endl;
  }

  // Set the member's current task.
  task->GetStatus().start();
  _member->SetTask(task);
}

void
Coordinator::
UpdateVersionMap(Agent* const _member, std::vector<Agent*> _agents) {
  for(Agent* agent : _agents){
    PlanningAgent* planningAgent = static_cast<PlanningAgent*>(agent);
    // Update each proximity agent's knowledge about this member's plan version.
    m_versionMap[_member][planningAgent] = planningAgent->GetPlanVersion();
  }
}

bool
Coordinator::
ValidateVersionMap(Agent* const _member, std::vector<Agent*> _agents) {
  for(Agent* agent : _agents){
    if(agent == _member)
      continue;
    PlanningAgent* planningAgent = static_cast<PlanningAgent*>(agent);
    if(m_debug)
        std::cout << "Actual Version Number for "
                  << agent->GetRobot()->GetLabel() << ": "
                  << planningAgent->GetPlanVersion() << std::endl
                  << _member->GetRobot()->GetLabel() << " stored version for "
                  << agent->GetRobot()->GetLabel() << ": "
                  << m_versionMap[_member][planningAgent]
                  << std::endl;
    if(m_versionMap[_member][planningAgent] != planningAgent->GetPlanVersion())
      return false;
  }
  return true;
}

double
Coordinator::
GetCurrentTime(){
  return m_currentTime;
}

bool
Coordinator::
IsClearToMoveOn(HandoffAgent* _agent){
  auto subtask = _agent->GetTask();
  if(!subtask)
    return true;
  //checks if agent is the one executing the subtask or the one coming to take
  //over the task
  if(_agent->IsPerformingSubtask()){
    auto wholeTask = m_tmpLibrary->GetTaskPlan()->GetWholeTask(subtask);
    auto index = std::find(wholeTask->m_subtasks.begin(),
      wholeTask->m_subtasks.end(), subtask);
    //checks if it is the last subtask in the whole task and thus no partner is
    //coming to take over the task
    if(index == wholeTask->m_subtasks.end()-1){
      wholeTask->m_task->GetStatus().complete();
      return true;
    }
    // Gets IT partner from the wholeTask
    auto it = std::distance(wholeTask->m_subtasks.begin(), index);
    auto partner = wholeTask->m_agentAssignment[it+1];

    // Next subtask is performed by the same agent, so no waiting for the IT is
    // needed
    if(partner == _agent)
      return true;

    // Lets the partner know that the first agent has arrived at the handoff
    //partner->SetClearToMove(true);
    //partner->SetPerformingSubtask(true);

    if(m_debug){
      std::cout << "Partner: " << partner->GetRobot()->GetLabel() << std::endl;
      std::cout << "Partner task: " << partner->GetTask() << std::endl;
    }
    // Checks if the other agent is there to hand the task off to
    if(!partner->GetTask() or partner->ReachedHandoff()){
      partner->SetPerformingSubtask(true);
      partner->SetClearToMove(true);
      // Allows the first agent to move on from the IT
      return true;
    }
  }
  // Indicates that the agent is the one receiving the handoff
  else {
    subtask = _agent->GetQueuedSubtasks().front();
    auto wholeTask = m_tmpLibrary->GetTaskPlan()->GetWholeTask(subtask);
    // Checks if the current task is a setup for the initial subtask in a whole
    // Task so there won't be a partner to take the task from
    if(wholeTask->m_subtasks[0] == subtask)
      return true;

    auto index = std::find(wholeTask->m_subtasks.begin(),
      wholeTask->m_subtasks.end(), subtask);
    auto it = std::distance(wholeTask->m_subtasks.begin(), index);
    //Checks if you were receiving the task from yourself
    auto partner = wholeTask->m_agentAssignment[it-1];

    if(partner == _agent)
      return true;
  }
  return false;
}

/*--------------------------- Initialize Functions ------------------------------*/

void
Coordinator::
InitializeAgents(){
  for(auto agent : m_memberAgents){
    agent->Initialize();
    agent->SetParentAgent(this);
  }
}

/*--------------------------- Helpers ------------------------------*/

std::vector<std::shared_ptr<MPTask>>
Coordinator::
ConvertActionsToTasks(std::vector<std::shared_ptr<Action>> _actionPlan){
/*
  WholeTask* wholeTask = m_tmpLibrary->GetTaskPlan()->GetWholeTasks()[0];

  std::vector<std::shared_ptr<MPTask>> taskPlan;
  // Convert each of the actions into appropriate tasks
  for(auto action : _actionPlan){
    auto robots = action->GetRobots();
    //Check if action is move or handoff
    auto startState = action->GetStartState();
    auto resultState = action->GetResultState();
    //Start Task
    if(startState.m_taskOwners.size() == 0 and resultState.m_taskOwners.size() ==1){
      std::cout << "Start Task" << std::endl;
      continue;
    }
    //Move Robot
    else if(startState.m_robotLocations.size() == 1){
      std::cout << "Move Robot: " << robots[0]->GetLabel() << std::endl;
      std::shared_ptr<MPTask> task = std::shared_ptr<MPTask>(new MPTask(robots[0]));

      auto boundaryIt = startState.m_robotLocations.find(robots[0]);
      auto start = boundaryIt->second;

      boundaryIt = resultState.m_robotLocations.find(robots[0]);
      auto goal = boundaryIt->second;

      //Non-Manipulator Constraints
      if(!robots[0]->IsManipulator()){
        auto radius = 1.2 * (robots[0]->GetMultiBody()->GetBoundingSphereRadius());

        std::unique_ptr<CSpaceBoundingBox> boundingBox(
            new CSpaceBoundingBox({start->GetRange(0).Center(),start->GetRange(1).Center(),0}));
        boundingBox->SetRange(0,(start->GetRange(0).Center()-radius),
                                (start->GetRange(0).Center()+radius));
        boundingBox->SetRange(1,(start->GetRange(1).Center()-radius),
                                (start->GetRange(1).Center()+radius));
        boundingBox->SetRange(2,-1,1);
        auto startConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(robots[0], std::move(boundingBox)));
        
				std::unique_ptr<CSpaceBoundingBox> boundingBox2(
            new CSpaceBoundingBox({goal->GetRange(0).Center(),goal->GetRange(1).Center(),0}));
        boundingBox2->SetRange(0,(goal->GetRange(0).Center()-radius/2),
                                (goal->GetRange(0).Center()+radius/2));
        boundingBox2->SetRange(1,(goal->GetRange(1).Center()-radius/2),
                                (goal->GetRange(1).Center()+radius/2));
        boundingBox2->SetRange(2,-1,1);
        auto goalConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(robots[0], std::move(boundingBox2)));

        task->SetStartConstraint(std::move(startConstraint));
        task->ClearGoalConstraints();
        task->AddGoalConstraint(std::move(goalConstraint));
      }
      //Manipulator Constraints
      else{
        auto startBox = start->Clone();
        if(m_debug){
          std::cout << "Start Type: " << startBox->Name() << std::endl;
        }
        auto startConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(m_robot, std::move(startBox)));


        auto goalBox = goal->Clone();
        if(m_debug){
          std::cout << "Goal Type: " << goalBox->Name() << std::endl;
        }
        auto goalConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(m_robot, std::move(goalBox)));


        task->SetStartConstraint(std::move(startConstraint));
        task->AddGoalConstraint(std::move(goalConstraint));
      }

      // Check if move robot is part of the whole task since handoff agent will
      // take of setup actions on its own
      if(startState.m_objectLocations.size() > 0){
        wholeTask->m_subtasks.push_back(task);
        HandoffAgent* agent = dynamic_cast<HandoffAgent*>(robots[0]->GetAgent());
        wholeTask->m_agentAssignment.push_back(agent);
        m_subtaskMap[task] = wholeTask;
        taskPlan.push_back(task);
      }

    }
    //Perform Handoff
    else if(startState.m_robotLocations.size() == 2 and m_debug){
      std::cout << "Hand off from: "
                << robots[0]->GetLabel()
                << " to: " << robots[1]->GetLabel()
                << std::endl;
    }

  }
  return taskPlan;
	*/
	return {};
}

void
Coordinator::
TMPAssignTasks(std::vector<std::shared_ptr<MPTask>> _taskPlan){
	/*
  for(auto& task : _taskPlan){
    HandoffAgent* agent = dynamic_cast<HandoffAgent*>(task->GetRobot()->GetAgent());
    agent->AddSubtask(task);
  }
  Simulation::GetStatClass()->SetStat("Subtasks", _taskPlan.size());
	*/
}

void
Coordinator::
DistributeTaskPlan(TaskPlan* _taskPlan){
	for(auto agent : m_memberAgents){
		for(auto& task : _taskPlan->GetAgentTasks(agent)){
			agent->AddSubtask(task);
		}
	}		
}

//TMPStrategyMethod*
std::string
Coordinator::
GetCurrentStrategy(){
	return m_tmpStrategyLabel;
}

TMPLibrary*
Coordinator::
GetTMPLibrary(){
	return m_tmpLibrary;
}

void
Coordinator::
SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph){
  *m_solution->GetRoadmap(m_robot) = *_graph;
}
