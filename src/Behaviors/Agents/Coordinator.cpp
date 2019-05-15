#include "Coordinator.h"

#include <limits>
#include <queue>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Behaviors/TMPStrategies/ITConnector.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "Behaviors/Agents/TaskBreakup.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Behaviors/TMPStrategies/Actions/Action.h"
#include "Behaviors/TMPStrategies/TMPHelperAlgorithms/EnforcedHillClimbing.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPLibrary/MPTools/TRPTool.h"
#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/InteractionInformation.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Simulator/Simulation.h"
#include "Simulator/BulletModel.h"
#include "Traits/CfgTraits.h"

#include "sandbox/gui/main_window.h"

/*------------------------------ Construction --------------------------------*/

Coordinator::
Coordinator(Robot* const _r) : Agent(_r) {
}


Coordinator::
Coordinator(Robot* const _r, XMLNode& _node) : Agent(_r) {

  // Parse the labels of the group members.
  for(auto& child : _node) {
    // Load the environment file used to create handoff templates
    if(child.Name() == "HandoffEnvironment") {
      // Ignore this node if we already have an environment.
      if(!m_handoffEnvironment)
        m_handoffEnvironment = std::unique_ptr<Environment>(new Environment(child));
    }
    else if(child.Name() == "PlacementMethod") {
      std::cout << "Reading Placement Method" << std::endl;
      AddPlacementMethod(PlacementMethod::Factory(m_robot->GetMPProblem(), child));
    }
    else {
      // Parse the robot label.
      const std::string memberLabel = child.Read("label", true, "",
          "The label of the member robot.");
      m_memberLabels.push_back(memberLabel);
    }
  }

  m_dmLabel = _node.Read("dmLabel", true, "", "Distance metric for checking "
      "nearest agents and charging locations.");

  m_tmp = _node.Read("tmp", false, false, "Does the coordinator use a tmp method?");

  m_it = _node.Read("it", false, true, "Generate the Capability and Combined Roadmaps.");

  m_connectionThreshold = _node.Read("connectionThreshold",true,1.2, 0., 1000.,
      "Acceptable variabliltiy in IT paths.");

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

  m_handoffTemplateRoadmap.reset(new RoadmapType(m_robot));

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);
  m_library->SetMPSolution(m_solution);

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

  GenerateDummyAgents();
  if(m_debug){
    std::cout << "Done Generating Dummy Agents" << std::endl;
  }

  GenerateHandoffTemplates();
  if(m_debug){
    std::cout << "Done Generating Handoff Templates" << std::endl;
  }

  m_megaRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);
  // Setting library task to set robot
  auto task = m_library->GetMPProblem()->GetTasks(m_robot).front();
  m_library->SetTask(task.get());
  /*
  TranslateHandoffTemplates();
  if(m_debug){
    std::cout << "Done Translating Handoff Templates" << std::endl;
  }

  *(m_handoffTemplateRoadmap.get()) = *m_megaRoadmap;

  //size_t id = Simulation::Get()->AddRoadmap(m_handoffTemplateRoadmap.get(),
  //    glutils::color(.4,.4,.4,.5));
  //m_simulatorGraphIDs.push_back(id);

  m_megaRoadmap->Write("postTranslate.map", m_robot->GetMPProblem()->GetEnvironment());

  SetupWholeTasks();
  if(m_debug){
    std::cout << "Done Setting up Whole Tasks" << std::endl;
  }

  m_megaRoadmap->Write("postSetupWholeTask.map", m_robot->GetMPProblem()->GetEnvironment());

  GenerateRoadmaps();
  if(m_debug){
    std::cout << "Done Generating Roadmaps" << std::endl;
  }
  */

  CreateCapabilityMaps();
  m_megaRoadmap->Write("postGenerateRoadaps.map", m_robot->GetMPProblem()->GetEnvironment());

  // Find group tasks plan with IT method
  if(!m_tmp){
    PlanWholeTasks();
    if(m_debug){
      std::cout << "Done Planning Whole Tasks" << std::endl;
    }

    m_megaRoadmap->Write("postPlanWholeTask.map", m_robot->GetMPProblem()->GetEnvironment());

    CopyCapabilityRoadmaps();
    if(m_debug){
      std::cout << "Done Copying Capability Roadmaps" << std::endl;
    }

    AssignInitialTasks();
    if(m_debug){
      std::cout << "Done Assigning Initial Tasks" << std::endl;
    }
  }
  // Find group taks plan with TMP method
  else{
    for(auto& robot : m_robot->GetMPProblem()->GetRobots()){
      robot->SetVirtual(true);
    }

    CopyCapabilityRoadmaps();
    if(m_debug){
      std::cout << "Done Copying Capability Roadmaps" << std::endl;
    }

    //Call TMP Methods
    std::vector<Robot*> robots;
    for(auto agent : m_memberAgents){
      robots.push_back(agent->GetRobot());
    }
    bool manip = false;
    if(m_robot->IsManipulator()){
      manip = true;
    }

    Simulation::GetStatClass()->StartClock("TMP FFRob");
    EnforcedHillClimbing tmp(m_capabilityRoadmaps,robots,m_wholeTasks[0]->m_task.get(),
                          m_solution->GetInteractionTemplates(), m_library, manip);
    std::vector<std::shared_ptr<Action>> actionPlan = tmp.Solve();
    Simulation::GetStatClass()->StopClock("TMP FFRob");

    if(m_debug){
      std::cout << "Printing action plan" << std::endl;
      for(auto& action : actionPlan){
        std::cout << action->PrintAction() << std::endl << std::endl;
      }
    }

    if(m_debug){
      std::cout << "Size of action plan" << actionPlan.size() << std::endl;
    }
    if(actionPlan.size() == 0){
      throw RunTimeException(WHERE, "Action Plan empty.");
    }

    if(m_debug){
      std::cout << "Done Solving TP Plan" << std::endl;
    }

    Simulation::GetStatClass()->StartClock("TMP Task Assignment");
    //Convert Actions to Tasks
    std::vector<std::shared_ptr<MPTask>> taskPlan = ConvertActionsToTasks(actionPlan);
    if(m_debug){
      std::cout << "Done Converting Actions to Tasks" << std::endl;
      std::cout << "Size of task plan" << taskPlan.size() << std::endl;
    }

    if(taskPlan.size() == 0){
      throw RunTimeException(WHERE, "TMP Task Plan empty.");
    }
    //Assign Tasks to Robots
    TMPAssignTasks(taskPlan);
    Simulation::GetStatClass()->StopClock("TMP Task Assignment");

    CopyCapabilityRoadmaps();
    if(m_debug){
      std::cout << "Done Copying Capability Roadmaps" << std::endl;
    }
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
  }

  if(m_debug){
    std::cout << "Looking at whole tasks now" << std::endl;
    for(auto wholeTask : m_wholeTasks){
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
  delete m_megaRoadmap;

  m_solution = nullptr;
  m_library  = nullptr;
  m_megaRoadmap = nullptr;

  for(auto task : m_wholeTasks){
    delete task;
  }

  for(auto id : m_simulatorGraphIDs){
    Simulation::Get()->RemoveRoadmap(id);
  }

  for(auto& graph : m_capabilityRoadmaps){
    delete graph.second;
  }
}

/*-------------------------- Coordinator Interface ---------------------------*/

std::shared_ptr<MPTask>
Coordinator::
AssignTask(std::shared_ptr<MPTask> _nextTask) {

  HandoffAgent* minAgent = nullptr;
  double minCost = std::numeric_limits<double>::max();

  // Generate the cost of a task for each agent
  // TODO: Thread this
  //std::vector<std::thread> costThreads(m_memberAgents.size());
  //auto lastAgent = GetLastAgent(m_subtaskMap[_nextTask]);
  for(auto agent : m_memberAgents){
    if(m_debug){
      std::cout << "Agent capability: " << agent->GetCapability() << std::endl;
      std::cout << "Task capability: " << _nextTask->GetCapability() << std::endl;
    }
    if(agent->GetCapability() != _nextTask->GetCapability() and _nextTask->GetCapability() != "")
      continue;
    //if(agent == lastAgent)
    //  continue;
    agent->GetRobot()->SetVirtual(false);
    agent->GenerateCost(_nextTask);
    agent->GetRobot()->SetVirtual(true);
    /*auto tempThread = std::thread([agent, _nextTask](){
      agent->GenerateCost(_nextTask);
    });
    costThreads.push_back(std::move(tempThread));*/
  }
  /*for(auto& thread : costThreads){
    thread.join();
  }*/

  // Assign the task to the agent with the lowest cost.
  for(auto agent : m_memberAgents){
    if(m_debug){
      std::cout << "Agent capability: " << agent->GetCapability() << std::endl;
      std::cout << "Task capability: " << _nextTask->GetCapability() << std::endl;
    }
    if(agent->GetCapability() != _nextTask->GetCapability() and _nextTask->GetCapability() != "")
      continue;
    if(m_debug){
      std::cout << agent->GetRobot()->GetLabel()
        << " has cost of "
        << agent->GetPotentialCost()
        << std::endl;
    }
    if(agent->GetPotentialCost() < minCost){
      minCost = agent->GetPotentialCost();
      minAgent = agent;
    }
  }
  if(m_debug){
    std::cout << "MinAgent: " << minAgent->GetRobot()->GetLabel() << std::endl;
  }
  // Assign subtask to min agent
  _nextTask->SetRobot(minAgent->GetRobot());
  //check if prior subtask was assigned to the same robot and merge them if so
  if(!m_subtaskMap[_nextTask]->m_agentAssignment.empty()){
    auto previousAgent = m_subtaskMap[_nextTask]->m_agentAssignment.back();
    if(previousAgent == minAgent){
      m_subtaskMap[_nextTask]->m_subtaskIterator--;
      auto lastTask = m_subtaskMap[_nextTask]->m_subtasks[m_subtaskMap[_nextTask]->m_subtaskIterator-1];
      lastTask->ClearGoalConstraints();
      for(auto& constraint : _nextTask->GetGoalConstraints()){
        lastTask->AddGoalConstraint(constraint->Clone());
      }
      m_subtaskMap[_nextTask]->m_subtasks.erase(m_subtaskMap[_nextTask]->m_subtasks.begin()
                                        + m_subtaskMap[_nextTask]->m_subtaskIterator);
    }
    else{
      m_subtaskMap[_nextTask]->m_agentAssignment.push_back(minAgent);
      minAgent->AddSubtask(_nextTask);
    }
  }
  else{
    m_subtaskMap[_nextTask]->m_agentAssignment.push_back(minAgent);
    minAgent->AddSubtask(_nextTask);
  }
  // See how long this agent will take to complete the subtask and if it
  // ends in a handoff add the next subtask to the queue
  double endTime = minAgent->GetTaskTime() + m_currentTime;
  std::shared_ptr<MPTask> newSubtask = GetNextSubtask(m_subtaskMap[_nextTask]);
  if(newSubtask){
    m_subtaskMap[newSubtask] = m_subtaskMap[_nextTask];
    newSubtask->SetEstimatedStartTime(endTime);
  }
  return newSubtask;
}

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
  if(!m_unassignedTasks.empty())
    return;
  for(auto wholeTask : m_wholeTasks){
    if(!wholeTask->m_task->GetStatus().is_complete())
      return;
  }
  for(auto agent : m_memberAgents){
    if(!agent->GetQueuedSubtasks().empty())
      return;
  }
  for(auto agent : m_memberAgents){
    agent->ClearVisualGraph();
  }
  if(m_debug){
    for(auto wholeTask : m_wholeTasks){
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

std::shared_ptr<MPTask>
Coordinator::
GetNextSubtask(WholeTask* _wholeTask){
  if(_wholeTask->m_subtaskIterator == _wholeTask->m_subtasks.size())
    return nullptr;
  return _wholeTask->m_subtasks[_wholeTask->m_subtaskIterator++];
}

HandoffAgent*
Coordinator::
GetLastAgent(WholeTask* _wholeTask){
  if(_wholeTask->m_subtaskIterator == 0)
    return nullptr;
  auto lastSubtask = _wholeTask->m_subtasks[_wholeTask->m_subtaskIterator - 1];
  return static_cast<HandoffAgent*>(lastSubtask->GetRobot()->GetAgent());
}

void
Coordinator::
AddSubtask(std::shared_ptr<MPTask> _subtask) {
  if(m_unassignedTasks.empty()){
    m_unassignedTasks.push_back(_subtask);
    return;
  }
  for(auto it = m_unassignedTasks.begin(); it != m_unassignedTasks.end(); it++){
    if(_subtask->GetEstimatedStartTime() < it->get()->GetEstimatedStartTime()){
      m_unassignedTasks.insert(it, _subtask);
      return;
    }
  }
  m_unassignedTasks.push_back(_subtask);
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
    auto wholeTask = m_subtaskMap[subtask];
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
    auto wholeTask = m_subtaskMap[subtask];
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

HandoffAgent*
Coordinator::
GetCapabilityAgent(std::string _capability){
  return m_dummyMap.at(_capability);
}


/*--------------------------- Initialize Functions ------------------------------*/

void
Coordinator::
GenerateDummyAgents(){
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : m_memberAgents){
    std::string capability = agent->GetCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      m_dummyMap[capability] = agent;
    }
  }
}

void
Coordinator::
GenerateHandoffTemplates(){
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  m_library->SetMPProblem(originalProblem);

  for(auto& info : originalProblem->GetInteractionInformations()){
    auto it = new InteractionTemplate(info.get());
    m_solution->AddInteractionTemplate(it);
    //FindITLocations(it);
  }

  std::cout << "Found Handoff Locations" << std::endl;

  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_handoffEnvironment));
  m_library->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;


  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){
    Simulation::GetStatClass()->StartClock("Construct InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());

    unusedAgents.clear();
    std::copy(m_memberAgents.begin(), m_memberAgents.end(), std::back_inserter(unusedAgents));
    auto handoffTasks = currentTemplate->GetInformation()->GetInteractionTasks();
    std::unordered_map<std::shared_ptr<MPTask>, HandoffAgent*> agentTasks;
    // Loop through all tasks and assign a robot of matching capability to the
    // task, then configuring the robot at the goal constraint.
    for(auto task : handoffTasks){
      m_library->SetTask(task.get());
      for(auto agent : unusedAgents){
        if(agent->GetCapability() == task->GetCapability()){
          agentTasks[task] = agent;
          Robot* tempRobot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
          task->SetRobot(tempRobot);
          unusedAgents.remove(agent);
          // Confiure tempRobot at the goal constraint for the task
          //        - Sample at the point of the goal constraint
          //        - Get CFG from sample and place tempRobot there
          auto boundingBox = task->GetGoalConstraints().front()->
              GetBoundary();
          std::vector<Cfg> goalPoints;

          MPSolution* sol = new MPSolution(m_robot);
          m_library->SetMPSolution(sol);
          m_library->SetTask(task.get());
          auto sampler = m_library->GetSampler("UniformRandomFree");
          size_t numNodes = 1, numAttempts = 100;
          tempRobot->SetVirtual(true);
          sampler->Sample(numNodes, numAttempts, boundingBox,
              std::back_inserter(goalPoints));

          tempRobot->SetVirtual(false);
          if(goalPoints.empty())
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");

          goalPoints[0].ConfigureRobot();
          break;
          /*

          std::vector<Cfg> goalPoints;
          auto sampler = m_library->GetSampler("UniformRandomFree");
          size_t numNodes = 1, numAttempts = 100;
          MPSolution* sol = new MPSolution(m_robot);
          m_library->SetMPSolution(sol);
          m_library->SetTask(task.get());

          if(m_robot->IsManipulator()){

            auto startBox = task->GetGoalConstraints()[0]->GetBoundary()->Clone();
            auto ptr = startBox.get();
            auto box = static_cast<CSpaceBoundingBox*>(ptr);
            auto ranges = box->GetRanges();
            for(size_t i = 0; i < 3; i++){
              auto range = ranges[i];
              auto center = range.Center();
              box->SetRange(i, center-.005, center+.005);
            }
            sampler->Sample(numNodes, numAttempts, box,
                std::back_inserter(goalPoints));
          }
          else{
            auto boundingBox = task->GetGoalConstraints()[0]->GetBoundary();
            sampler->Sample(numNodes, numAttempts, boundingBox,
                std::back_inserter(goalPoints));
          }

          if(goalPoints.empty())
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");

          goalPoints[0].ConfigureRobot();
          break;

          */
        }
      }
    }
    // Set the unused agents to virtual before planning.
    for(auto agent : unusedAgents){
      auto robot = problemCopy->GetRobot(agent->GetRobot()->GetLabel());
      robot->SetVirtual(true);
    }
    int check = 0;
    for(auto task : handoffTasks){
      Robot* taskRobot = problemCopy->GetRobot(agentTasks[task]->GetRobot()->GetLabel());
      std::unique_ptr<MPSolution> handoffSolution(new MPSolution(taskRobot));
      // Store the current configuration of the robot, since the multibody
      // will be moved while solving.
      auto currentConfig = taskRobot->GetMultiBody()->GetCurrentDOFs();
      if(m_debug){
        for(auto& robot : problemCopy->GetRobots()){
          std::cout << robot->GetLabel() << " - " << robot.get()
                    << ": " << robot->GetMultiBody()->GetCurrentDOFs()
                    << " - " << robot->IsVirtual() << std::endl;
        }
      }

      task->SetRobot(taskRobot);
      // Solve for non-mainpulator robot teams
      if(!taskRobot->IsManipulator()){
        m_library->Solve(problemCopy.get(), task.get(), handoffSolution.get());
      }
      // Solve for manipulator robot teams
      else {
        std::vector<Cfg> startPoints;
        MPSolution* sol = new MPSolution(m_robot);
        m_library->SetMPSolution(sol);
        auto sampler = m_library->GetSampler("UniformRandomFree");
        size_t numNodes = 1, numAttempts = 100;
        auto boundingBox = task->GetStartConstraint()->GetBoundary();
        m_library->SetTask(task.get());
        sampler->Sample(numNodes, numAttempts, boundingBox,
            std::back_inserter(startPoints));

        if(startPoints.empty())
          throw RunTimeException(WHERE, "No valid start handoff position for the robot.");

        startPoints[0].ConfigureRobot();
        std::cout << startPoints[0].PrettyPrint() << std::endl;

        m_library->Solve(problemCopy.get(), task.get(),
            handoffSolution.get(), "FixedPRM", LRand(), "FixedPRM");
      }

      taskRobot->GetMultiBody()->Configure(currentConfig);

      if(m_debug){
        std::cout << "Size of path: " << handoffSolution->GetPath()->Cfgs().size() << std::endl;
        for(auto cfg : handoffSolution->GetPath()->Cfgs()){
          std::cout << cfg.PrettyPrint() << std::endl;
        }
      }


      handoffSolution->GetRoadmap()->Write("indHandoffTemplate" + std::to_string(check) + ".map", problemCopy->GetEnvironment());
      check++;

      // Store the roadmap for each task in the handoff
      auto rob = handoffSolution->GetRoadmap()->GetRobot();
      handoffSolution->GetRoadmap()->SetRobot(originalProblem->GetRobot(rob->GetLabel()));
      currentTemplate->AddRoadmap(handoffSolution->GetRoadmap());
      currentTemplate->AddPath(handoffSolution->GetPath()->Cfgs(), originalProblem);

      if(currentTemplate->GetInformation()->SavedPaths()){
        //currentTemplate->AddPath(handoffSolution->GetPath()->Cfgs()),
           // handoffSolution->GetPath()->Length());
        std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
        currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
        std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
      }
      else{
        // Add final configuration of path to template
        std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
        currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
        std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
      }
    }


    currentTemplate->ConnectRoadmaps(m_robot, originalProblem);

    Simulation::GetStatClass()->StopClock("Construct InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());
    Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Vertices", currentTemplate->GetConnectedRoadmap()->get_num_vertices());
    Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
                            +"::Edges", currentTemplate->GetConnectedRoadmap()->get_num_vertices());

    size_t count = 0;
    for(auto rm : currentTemplate->GetRoadmaps()){
      count++;
      Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Vertices", rm->get_num_vertices());
      Simulation::GetStatClass()->SetStat(currentTemplate->GetInformation()->GetLabel()
          +"::"+std::to_string(count)
          +"::Edges", rm->get_num_vertices());
    }

    std::cout << "Trying to write handoffTemplate Map" << std::endl;
    currentTemplate->GetConnectedRoadmap()->Write("handoffTemplate.map",
        problemCopy->GetEnvironment());

    // Reset the agents to non-virtual, since they could be used in the next
    // template.
    for(auto agent : unusedAgents){
      problemCopy->GetRobot(agent->GetRobot()->GetLabel())->SetVirtual(false);
    }
  }


  m_library->SetMPProblem(originalProblem);
  m_library->SetMPSolution(m_solution);
  m_library->SetTask(originalProblem->GetTasks(m_robot)[0].get());
}

void
Coordinator::
TranslateHandoffTemplates() {

  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  m_library->SetMPProblem(originalProblem);

  //for(auto& it : m_solution->GetInteractionTemplates()){
    //m_solution->AddInteractionTemplate(it);
    //FindITLocations(it.get());
  //}

  std::cout << "Found Handoff Locations" << std::endl;
  Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
  auto vcm = m_library->GetValidityChecker("terrain_solid");
  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){

    if(m_debug){
      auto g = currentTemplate->GetConnectedRoadmap();
      std::cout << "Original handoff position" << std::endl;
      for(auto vit = g->begin(); vit!=g->end(); vit++){
        std::cout << vit->property().PrettyPrint() << std::endl;
      }
    }

    Simulation::GetStatClass()->StartClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
    for(auto centerCfg : currentTemplate->GetInformation()->GetTemplateLocations()){
      Simulation::GetStatClass()->StartClock("Placement InteractionTemplate "
                + currentTemplate->GetInformation()->GetLabel());

      RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = currentTemplate->GetConnectedRoadmap();

      // Copy vertices and map the change in VIDs.
      std::unordered_map<VID, VID> oldToNew;
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        const VID oldVID = vit->descriptor();
        auto relativeCfg = vit->property();
        relativeCfg.TransformCfg(centerCfg);

        //bool isValid = vcm->IsValid(relativeCfg, "ValidateITCfg");
        //if(isValid){
          const VID newVID = m_megaRoadmap->AddVertex(relativeCfg);
          oldToNew[oldVID] = newVID;
        //}
      }

      // Keep track of the distinct transformed handoff roadmaps
      for(auto distinctRoadmap : currentTemplate->GetDistinctRoadmaps()) {
        std::vector<size_t> transformedRoadmap;
        for(auto vid : distinctRoadmap) {
          transformedRoadmap.push_back(oldToNew[vid]);
        }
        m_transformedRoadmaps.push_back(transformedRoadmap);
      }


      if(m_debug){
        auto r = m_transformedRoadmaps.front();
        std::cout << "Transformed Postion of Roadmap" << std::endl;
        for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
          std::cout << vit->descriptor() << std::endl;
          std::cout << vit->property().PrettyPrint() << std::endl;
        }
        for(auto v : r){
          std::cout << m_megaRoadmap->GetVertex(v).PrettyPrint() << std::endl;
        }
      }

      m_megaRoadmap->InstallHook(RoadmapType::HookType::AddEdge, "debug",
          [](RoadmapType::EI _ei) {
          if(_ei->property().GetWeight()==0){
          std::cout << "Zero weight edge" << std::endl;
          }
          });
      // Copy edges into the mega roadmap
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        for(auto eit = vit->begin(); eit != vit->end(); ++eit) {
          const VID source = oldToNew[eit->source()];
          const VID target = oldToNew[eit->target()];
          if(!m_megaRoadmap->IsEdge(source, target)){
            // Call translate cfg on the all the intermediates and built
            // up a new vector of intermediates to store in the edge property
            // before storing it in the megaRoadmap
            std::vector<Cfg> intermediates = eit->property().GetIntermediates();
            for(auto cfg : intermediates){
              cfg.TransformCfg(centerCfg);
            }
            m_megaRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
      m_megaRoadmap->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
    }
    Simulation::GetStatClass()->StopClock("Placement InteractionTemplate "
              + currentTemplate->GetInformation()->GetLabel());
  }
  //Get rid of cfgs that are invalid after placement
  /*for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
    auto cfg = vit->property();
    auto vcm = m_library->GetValidityChecker("terrain_solid");
    bool isValid = vcm->IsValid(cfg, "ValidateITCfg");
    if(!isValid){
      m_megaRoadmap->DeleteVertex(vit->descriptor());
    }
  }*/


  Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
}

void
Coordinator::
SetupWholeTasks(){
  for(auto task : m_robot->GetMPProblem()->GetTasks(m_robot)){
    if(task->GetGoalConstraints().size() == 0)
      continue;
    WholeTask* wholeTask = new WholeTask();
    wholeTask->m_task = task;
    for(auto const& elem : m_dummyMap){
      wholeTask->m_startPoints[elem.first] = {};
      wholeTask->m_goalPoints[elem.first] = {};
      wholeTask->m_startVIDs[elem.first] = {};
      wholeTask->m_goalVIDs[elem.first] = {};
    }

    // find a start and goal configuration for the coordinator
    m_library->SetTask(task.get());
    auto startBox = task->GetStartConstraint()->GetBoundary();
    std::vector<Cfg> startPoints;
    auto sampler = m_library->GetSampler("UniformRandomFree");
    size_t numNodes = 1, numAttempts = 100;
    sampler->Sample(numNodes, numAttempts, startBox,
        std::back_inserter(startPoints));

    if(startPoints.empty())
      throw RunTimeException(WHERE, "No valid initial position for the robot.");

    auto goalBox = task->GetGoalConstraints().front()->GetBoundary();
    std::vector<Cfg> goalPoints;
    sampler->Sample(numNodes, numAttempts, goalBox,
        std::back_inserter(goalPoints));

    if(goalPoints.empty())
      throw RunTimeException(WHERE, "No valid goal position for the robot.");

    wholeTask->m_startPoints[m_robot->GetLabel()] = {startPoints[0]};
    wholeTask->m_goalPoints[m_robot->GetLabel()] = {goalPoints[0]};

    // Loop through each type of capability then push start/goal constraints
    // into vectors in WholeTask
    for(auto const& elem : m_dummyMap) {
      // Set library robot to the corresponding capability
      task->SetRobot(elem.second->GetRobot());
      m_library->SetTask(task.get());
      // Sample to find valid start and goal points in the environment
      auto startBox = task->GetStartConstraint()->GetBoundary();
      std::vector<Cfg> startPoints;
      auto sampler = m_library->GetSampler("UniformRandomFreeTerrain");
      size_t numNodes = 1, numAttempts = 100;
      try{
      sampler->Sample(numNodes, numAttempts, startBox,
          std::back_inserter(startPoints));
      }
      catch(...){
        std::cout << "Didn't find any valid configurations for "
          + elem.second->GetRobot()->GetCapability() << std::endl;
      }

      if(!startPoints.empty())
        wholeTask->m_startPoints[elem.second->GetCapability()].push_back(startPoints[0]);

      auto goalBox = task->GetGoalConstraints().front()->GetBoundary();
      std::vector<Cfg> goalPoints;
      sampler->Sample(numNodes, numAttempts, goalBox,
          std::back_inserter(goalPoints));

      if(!goalPoints.empty())
        wholeTask->m_goalPoints[elem.second->GetCapability()].push_back(goalPoints[0]);

    }

    task->SetRobot(m_robot);
    m_library->SetTask(task.get());

    // Create 0 weight edges between each capability and the coordinator
    // configuration.
    auto coordinatorStartVID = m_megaRoadmap->AddVertex(
                    wholeTask->m_startPoints[m_robot->GetLabel()][0]);

    wholeTask->m_startVIDs[m_robot->GetLabel()] = {coordinatorStartVID};

    auto coordinatorGoalVID = m_megaRoadmap->AddVertex(wholeTask->m_goalPoints[m_robot->GetLabel()][0]);

    wholeTask->m_goalVIDs[m_robot->GetLabel()] = {coordinatorGoalVID};

    const DefaultWeight<Cfg> weight;

    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    for(auto const& elem : wholeTask->m_startPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto start : elem.second) {
        auto agentStartVID = m_megaRoadmap->AddVertex(start);

        // Add the start points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_megaRoadmap->InstallHook(RoadmapType::HookType::AddEdge, "debug",
            [](RoadmapType::EI _ei) {
            if(_ei->property().GetWeight()==0){
            std::cout << "Zero weight edge" << std::endl;
            }
            });
        m_wholeTaskStartEndPoints.push_back({agentStartVID});
        wholeTask->m_startVIDs[elem.first].push_back(agentStartVID);
        m_megaRoadmap->AddEdge(coordinatorStartVID, agentStartVID, {weight,weight});
        m_megaRoadmap->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
      }

    }

    for(auto const& elem : wholeTask->m_goalPoints){
      if(elem.first == m_robot->GetLabel())
        continue;

      for(auto goal : elem.second) {
        auto agentGoalVID = m_megaRoadmap->AddVertex(goal);

        // Add the end points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_megaRoadmap->InstallHook(RoadmapType::HookType::AddEdge, "debug",
            [](RoadmapType::EI _ei) {
            if(_ei->property().GetWeight()==0){
            std::cout << "Zero weight edge" << std::endl;
            }
            });
        m_wholeTaskStartEndPoints.push_back({agentGoalVID});
        wholeTask->m_goalVIDs[elem.first].push_back(agentGoalVID);
        m_megaRoadmap->AddEdge(coordinatorGoalVID, agentGoalVID, {weight,weight});
        m_megaRoadmap->RemoveHook(RoadmapType::HookType::AddEdge, "debug");
      }

    }
    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
    m_wholeTasks.push_back(wholeTask);
  }
  m_library->SetTask(m_robot->GetMPProblem()->GetTasks(m_robot)[0].get());
  //TODO Find a better way to do this and a better home
  std::unordered_map<Cfg*,std::vector<Cfg>*> interactionPoints;
  for(auto& currentTemplate : m_solution->GetInteractionTemplates()){
    for(auto& path : currentTemplate->GetTranslatedPaths()){
      Cfg& interactionPoint = path[path.size()-1];
      interactionPoints[&interactionPoint] = &path;
    }
  }

  for(auto wholeTask : m_wholeTasks){
    wholeTask->m_interactionPoints = interactionPoints;
  }
}


void
Coordinator::
GenerateRoadmaps() {
  for(auto& r : m_robot->GetMPProblem()->GetRobots()){
    r->SetVirtual(true);
  }

  //Add robot start locations if using tmp method
  if(m_tmp){
    for(auto agent : m_memberAgents){
      auto robot = agent->GetRobot();
      auto cfg = robot->GetSimulationModel()->GetState();
      auto capabilityRobot = m_dummyMap[robot->GetCapability()]->GetRobot();
      cfg.SetRobot(capabilityRobot);
      auto vid = m_megaRoadmap->AddVertex(cfg);
      m_wholeTaskStartEndPoints.push_back({vid});
    }
  }

  for(auto const& elem : m_dummyMap){
    auto capability = elem.first;
    Simulation::GetStatClass()->StartClock("Construct CapabilityMap " + capability);
    HandoffAgent* dummyAgent = elem.second;
    dummyAgent->InitializeRoadmap();

    std::unordered_map<size_t, size_t> handoffVIDMap;
    std::unordered_map<size_t, size_t> inverseVIDMap;
    auto graph = dummyAgent->GetMPSolution()->GetRoadmap();

    Simulation::GetStatClass()->StartClock("Copy Handoffs to CapabilityMap " + capability);
    // Copy cfgs of handoffs of same capability into corresponding capability map

    for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
      auto roadmap = m_transformedRoadmaps[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < roadmap.size(); j++){
          auto vid = roadmap[j];
          const auto newVID = graph->AddVertex(m_megaRoadmap->GetVertex(vid));
          handoffVIDMap[newVID] = vid;
          inverseVIDMap[vid] = newVID;
        }
      }
    }
    //Adding edges of handoffs
    for(auto vit = graph->begin(); vit != graph->end(); vit++){
      auto vid1 = vit->descriptor();
      auto orig1 = handoffVIDMap[vid1];
      for(auto vit2 = graph->begin(); vit2 != graph->end(); vit2++){
        if(vit == vit2) continue;
        auto vid2 = vit2->descriptor();
        auto orig2 = handoffVIDMap[vid2];
        if(m_megaRoadmap->IsEdge(orig1, orig2)){
          auto edge = m_megaRoadmap->GetEdge(orig1, orig2);
          graph->AddEdge(vid1, vid2, edge);
        }

      }
    }

    Simulation::GetStatClass()->StopClock("Copy Handoffs to CapabilityMap " + capability);

    if(m_debug){
      std::cout << "Done initializing capability map with corresponding cfgs from megaRoadmap"
                << std::endl;
    }

    // Connect handoff templates of the same capability.
    for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
      auto roadmap = m_transformedRoadmaps[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = i+1; j < m_transformedRoadmaps.size(); j++){
          auto roadmap2 = m_transformedRoadmaps[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent);
          }
        }
      }
    }

    if(m_debug){
      std::cout << "Done connecting handoff templates of same capability" << std::endl;
    }

    // Attempt to connect the whole task start/end points to each
    // other in the capability maps
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
            GetAgent()->GetCapability()){
        for(size_t j = 0; j < roadmap.size(); j++){
          auto vid = roadmap[j];
          const auto newVID = graph->AddVertex(m_megaRoadmap->GetVertex(vid));
          handoffVIDMap[newVID] = vid;
          inverseVIDMap[vid] = newVID;
        }
      }
    }

    // Attempt to connect the whole task start/end points to the
    // ITs in the capability map
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < m_transformedRoadmaps.size(); j++){
          auto roadmap2 = m_transformedRoadmaps[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent);
          }
        }
      }
    }

    // Attempt to connect the start/end points
    for(size_t i = 0; i < m_wholeTaskStartEndPoints.size(); i++){
      auto roadmap = m_wholeTaskStartEndPoints[i];
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        for(size_t j = 0; j < m_wholeTaskStartEndPoints.size(); j++){
          if(i == j) continue;
          auto roadmap2 = m_wholeTaskStartEndPoints[j];
          if(capability == m_megaRoadmap->GetVertex(roadmap2[0]).GetRobot()->
              GetAgent()->GetCapability()){
            // Find point in both roadmaps and try to connect them
            ConnectDistinctRoadmaps(roadmap, roadmap2, dummyAgent);
          }
        }
      }
    }

    m_capabilityRoadmaps[capability] = graph;

    Simulation::GetStatClass()->StopClock("Construct CapabilityMap " + capability);
    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    // Copy over newly found vertices
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      // Add vertices found in building the capability map to the mega roadmap
      // without copying over the handoff vertices that already exist
      if(handoffVIDMap.find(vit->descriptor()) == handoffVIDMap.end()){
        auto megaVID = m_megaRoadmap->AddVertex(vit->property());
        handoffVIDMap[vit->descriptor()] = megaVID;
      }
    }

    if(m_debug){
      std::cout << "Done copying over vertices" << std::endl;
    }

    // Copy over newly found edges in capability to mega roadmap
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      for(auto eit = vit->begin(); eit != vit->end(); ++eit){
        auto source = handoffVIDMap[eit->source()];
        auto target = handoffVIDMap[eit->target()];
        // Won't copy over existing edges to the mega roadmap
        if(!m_megaRoadmap->IsEdge(source, target)){
            m_megaRoadmap->AddEdge(source, target, eit->property());
        }
      }
    }

    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");
    if(m_debug){
      std::cout << "Done copying over edges" << std::endl;
    }
  }

  //Printing megaroadmap
  if(m_debug){
    std::cout << "Printing m_megaRoadmap after constructing capability maps" << std::endl;
    for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
      std::cout << vit->property().PrettyPrint() << std::endl;
    }
  }

  for(auto& r : m_robot->GetMPProblem()->GetRobots()){
    if(r.get() != m_robot){
      r->SetVirtual(true);
    }
  }

}

void
Coordinator::
PlanWholeTasks() {
  // Not so efficient, just making a full copy for now until we expand the
  // MPSolution API.
  //*m_solution->GetRoadmap(m_robot) = *m_megaRoadmap;

  //TODO::Probably don't need intitial task
  auto initTask = m_robot->GetMPProblem()->GetTasks(m_robot)[0];
  m_library->SetTask(initTask.get());

  m_library->Solve(m_robot->GetMPProblem(), initTask.get(), m_solution, "EvaluateMapStrategy",
      LRand(), "InitTask");

  m_solution->SetRoadmap(m_robot, m_megaRoadmap);

  if(m_debug){
    Simulation::Get()->AddRoadmap(m_megaRoadmap,
      glutils::color(1., 0., 1., 0.2));
  }
  //Find path for each whole task in megaRoadmap
  for(auto wholeTask: m_wholeTasks){
    Simulation::GetStatClass()->StartClock("IT MegaRoadmap Query");
    wholeTask->m_task->SetRobot(m_robot);
    m_library->SetTask(wholeTask->m_task.get());
    auto& c = wholeTask->m_task->GetGoalConstraints()[0];
    c->Clone();
    m_library->Solve(m_robot->GetMPProblem(), wholeTask->m_task.get(), m_solution, "EvaluateMapStrategy",
      LRand(), "PlanWholeTask");
    //Save cfg path in the handoff class
    wholeTask->m_wholePath = m_solution->GetPath()->Cfgs();
    Simulation::GetStatClass()->StopClock("IT MegaRoadmap Query");
    //TODO:: Need to update for multiple tasks
    Simulation::GetStatClass()->SetStat("WholePathLength", m_solution->GetPath()->Length());
  }

  TaskBreakup tb(m_robot);

  for(auto& wholeTask : m_wholeTasks){
    tb.BreakupTask(wholeTask);
    Simulation::GetStatClass()->StopClock("IT Task Decomposition");
    Simulation::GetStatClass()->SetStat("Subtasks", wholeTask->m_subtasks.size());
  }
}

void
Coordinator::
InitializeAgents(){
  for(auto agent : m_memberAgents){
    agent->Initialize();
    agent->SetParentAgent(this);
  }
}

void
Coordinator::
CopyCapabilityRoadmaps(){
  for(auto agent : m_memberAgents){
    //Copy corresponding capability roadmap into agent
    auto graph = m_capabilityRoadmaps[agent->GetCapability()];
    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    g->SetRobot(agent->GetRobot());
    agent->SetRoadmapGraph(g);

    // Write the capability map.
    graph->Write(agent->GetRobot()->GetLabel() + ".map",
        m_robot->GetMPProblem()->GetEnvironment());
  }
}

void
Coordinator::
AssignInitialTasks() {
  // Load m_unassignedTasks with the initial subtasks for all tasks.
  Simulation::GetStatClass()->StartClock("IT Task Assignment");
  for(auto wholeTask : m_wholeTasks){
    auto subtask = GetNextSubtask(wholeTask);
    if(subtask){
      m_unassignedTasks.push_back(subtask);
      m_subtaskMap[subtask] = wholeTask;
    }
  }
  for(auto agent : m_memberAgents){
    agent->GetRobot()->SetVirtual(true);
  }
  // Assign all of the tasks (and their subtasks) to different agents.
  int numSubTs = 0;
  while(!m_unassignedTasks.empty()){
    std::cout << "SubTask: " << numSubTs << std::endl;
    numSubTs++;
    std::shared_ptr<MPTask> nextTask = m_unassignedTasks.front();
    auto newSubtask = AssignTask(nextTask);
    m_unassignedTasks.pop_front();
    if(newSubtask){
      AddSubtask(newSubtask);
    }
  }
  for(auto agent : m_memberAgents){
    agent->GetRobot()->SetVirtual(false);
  }
  Simulation::GetStatClass()->StopClock("IT Task Assignment");
}

/*--------------------------- Helpers ------------------------------*/

void
Coordinator::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2]*PI;

  double newX = x*cos(theta) - y*sin(theta);
  double newY = x*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX, newY, oldTheta});

  _relativeCfg += _centerCfg;
}

void
Coordinator::
ConnectDistinctRoadmaps(vector<size_t> _roadmap1, vector<size_t> _roadmap2,
    HandoffAgent* _agent) {
  std::cout << "Connecting Distinct Roadmaps" << std::endl;
  Robot* dummyRobot = _agent->GetRobot();
  auto originalProblem = m_library->GetMPProblem();
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*dummyRobot->GetMPProblem()));
  // Loop through each roadmap until valid cfgs from both are found.

  //TODO: Check if the roadmaps are reachable from the other
  //For now, just select random cfgs from each and try to connect
  for(auto vid1 : _roadmap1) {
    Cfg cfg1 = m_megaRoadmap->GetVertex(vid1);
    auto cfg1Copy = cfg1;
    cfg1Copy.SetRobot(dummyRobot);
    if(!m_library->GetValidityChecker("terrain_solid")->IsValid(cfg1Copy, "cfg1"))
      continue;

    for(auto vid2 : _roadmap2) {
      Cfg cfg2 = m_megaRoadmap->GetVertex(vid2);
      auto cfg2Copy = cfg2;
      cfg2Copy.SetRobot(dummyRobot);
      if(vid1 == vid2 || cfg1Copy == cfg2Copy) continue;
      if(!m_library->GetValidityChecker("terrain_solid")->IsValid(cfg2Copy, "cfg2"))
        continue;

      // Attempt to plan between the two roadmaps
      MPTask* tempTask = new MPTask(dummyRobot);
      std::unique_ptr<CSpaceConstraint> start(new CSpaceConstraint(dummyRobot, cfg1Copy));
      std::unique_ptr<CSpaceConstraint> goal(new CSpaceConstraint(dummyRobot, cfg2Copy));
      tempTask->SetStartConstraint(std::move(start));
      tempTask->AddGoalConstraint(std::move(goal));

      if(m_debug){
        std::cout << "Calling solve" << std::endl;
      }
      // Solve for manipulator robot team
      if(m_robot->IsManipulator()){
        m_library->Solve(problemCopy.get(), tempTask, _agent->GetMPSolution(),
            "EvaluateMapStrategy", LRand(), "ConnectingDistinctRoadmaps");
      }
      // Solve for non-manipulator robot team
      else{
        m_library->Solve(problemCopy.get(), tempTask, _agent->GetMPSolution(),
            "LazyPRM", LRand(), "ConnectingDistinctRoadmaps");
      }

      if(m_debug){
        std::cout << "Finish solving" << std::endl;
      }

      if(!_agent->GetMPSolution()->GetPath()->Cfgs().empty()) {
        if(m_debug){
          std::cout << "Finishing connecting distinct roadmaps" << std::endl;
        }
        m_library->SetMPProblem(originalProblem);
        return;
      }
    }
  }
  m_library->SetMPProblem(originalProblem);
}

std::vector<std::shared_ptr<MPTask>>
Coordinator::
ConvertActionsToTasks(std::vector<std::shared_ptr<Action>> _actionPlan){

  WholeTask* wholeTask = m_wholeTasks[0];

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

        /*std::unique_ptr<CSpaceBoundingSphere> boundingSphere(
            new CSpaceBoundingSphere(start->GetCenter(), radius));
        auto startConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(robots[0], std::move(boundingSphere)));
*/
        std::unique_ptr<CSpaceBoundingBox> boundingBox(
            new CSpaceBoundingBox({start->GetRange(0).Center(),start->GetRange(1).Center(),0}));
        boundingBox->SetRange(0,(start->GetRange(0).Center()-radius),
                                (start->GetRange(0).Center()+radius));
        boundingBox->SetRange(1,(start->GetRange(1).Center()-radius),
                                (start->GetRange(1).Center()+radius));
        boundingBox->SetRange(2,-1,1);
        auto startConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(robots[0], std::move(boundingBox)));

       /* std::unique_ptr<CSpaceBoundingSphere> boundingSphere2(
            new CSpaceBoundingSphere(goal->GetCenter(), radius));
        auto goalConstraint = std::unique_ptr<BoundaryConstraint>
          (new BoundaryConstraint(robots[0], std::move(boundingSphere2)));
*/
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
}

void
Coordinator::
TMPAssignTasks(std::vector<std::shared_ptr<MPTask>> _taskPlan){
  for(auto& task : _taskPlan){
    HandoffAgent* agent = dynamic_cast<HandoffAgent*>(task->GetRobot()->GetAgent());
    agent->AddSubtask(task);
  }
  Simulation::GetStatClass()->SetStat("Subtasks", _taskPlan.size());
}

void
Coordinator::
FindITLocations(InteractionTemplate* _it){
  /*if(m_robot->GetMultiBody()->GetBaseType() == Body::Type::Fixed){

  }
  else {
    std::cout << "Calling placement methods" << std::endl;
    //TODO::Implement check/call for disjoint and overlapping workspaces
    //m_ITPlacementMethods["ow"]->PlaceIT(_it, m_solution, m_library);
    m_ITPlacementMethods["djw"]->PlaceIT(_it, m_solution, m_library, this);
    m_ITPlacementMethods["ow"]->PlaceIT(_it, m_solution, m_library, this);
    std::cout << "Used placement methods" << std::endl;
  }*/
  for(auto& method : m_ITPlacementMethods){
    Simulation::GetStatClass()->StartClock("Placing Templates with: " + method.second->GetLabel());
    method.second->PlaceIT(_it, m_solution, m_library, this);
    Simulation::GetStatClass()->StopClock("Placing Templates with: " + method.second->GetLabel());
  }
}

void
Coordinator::
AddPlacementMethod(std::unique_ptr<PlacementMethod> _pm){
  m_ITPlacementMethods[_pm->GetLabel()] = std::move(_pm);
}

void
Coordinator::
CreateCapabilityMaps(){
  for(auto agent : m_memberAgents){
    agent->GetRobot()->SetVirtual(true);
  }
  std::cout << "Finding Handoff Locations" << std::endl;
  auto originalProblem = m_robot->GetMPProblem();
  m_library->SetMPProblem(originalProblem);

  for(auto& it : m_solution->GetInteractionTemplates()){
    FindITLocations(it.get());
  }

  std::cout << "Found Handoff Locations" << std::endl;

  TranslateHandoffTemplates();
  auto tempG = m_megaRoadmap;
  tempG->Write("CoordinatorJustTemplates.map", m_robot->GetMPProblem()->GetEnvironment());
  std::cout << "Writing template map." << std::endl;
  SetupWholeTasks();

  if(!m_robot->IsManipulator()){
    for(auto agent : m_memberAgents){
      auto robot = agent->GetRobot();
      auto cfg = robot->GetSimulationModel()->GetState();
      auto capabilityRobot = m_dummyMap[agent->GetCapability()]->GetRobot();
      cfg.SetRobot(capabilityRobot);
      auto vid = m_megaRoadmap->AddVertex(cfg);
      m_wholeTaskStartEndPoints.push_back({vid});
    }
  }

  if(!m_it)
    return;

  ITConnector connector(m_connectionThreshold,m_library);
  auto dm = m_library->GetDistanceMetric(m_dmLabel);

  std::vector<Cfg> startAndGoal;
  for(auto vid : m_wholeTaskStartEndPoints){
    startAndGoal.push_back(m_megaRoadmap->GetVertex(vid[0]));
  }

  for(auto it = m_dummyMap.begin(); it != m_dummyMap.end(); it++){
    const std::string capability = it->first;

    Simulation::GetStatClass()->StartClock("ConstructCapabilityMap::"+capability);
    auto graph = connector.ConnectInteractionTemplates(
                          m_solution->GetInteractionTemplates(),
                          capability,
                          startAndGoal,
                          m_megaRoadmap);

    Simulation::GetStatClass()->StopClock("ConstructCapabilityMap::"+capability);
    auto robot = it->second->GetRobot();
    graph->Write("CoordinatorTemplates.map", robot->GetMPProblem()->GetEnvironment());


    Simulation::GetStatClass()->StartClock("Construction MegaRoadmap");
    // Copy over newly found vertices
    std::unordered_map<size_t,size_t> handoffVIDMap;
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      // Add vertices found in building the capability map to the mega roadmap
      // without copying over the handoff vertices that already exist
      if(handoffVIDMap.find(vit->descriptor()) == handoffVIDMap.end()){
        auto megaVID = m_megaRoadmap->AddVertex(vit->property());
        handoffVIDMap[vit->descriptor()] = megaVID;
      }
    }

    if(m_debug){
      std::cout << "Done copying over vertices" << std::endl;
    }

    typedef RoadmapGraph<Cfg,DefaultWeight<Cfg>> RoadmapType;
    graph->InstallHook(RoadmapType::HookType::AddEdge, "debug",
        [](RoadmapType::EI _ei) {
        if(_ei->property().GetWeight()==0){
          std::cout << "Zero weight edge" << std::endl;
        }
        });
    m_capabilityRoadmaps[capability] = graph;
    // Copy over newly found edges in capability to mega roadmap
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      for(auto eit = vit->begin(); eit != vit->end(); ++eit){
        auto source = handoffVIDMap[eit->source()];
        auto target = handoffVIDMap[eit->target()];
        // Won't copy over existing edges to the mega roadmap
        if(!m_megaRoadmap->IsEdge(source, target)){
          m_megaRoadmap->AddEdge(source, target, eit->property());
        }
      }
    }

    graph->RemoveHook(RoadmapType::HookType::AddEdge, "debug");

    Simulation::GetStatClass()->StopClock("Construction MegaRoadmap");

    m_megaRoadmap->Write("MegaTemplates.map", robot->GetMPProblem()->GetEnvironment());
    Simulation::GetStatClass()->SetStat("MegaRoadMap::Vertices", m_megaRoadmap->get_num_vertices());
    Simulation::GetStatClass()->SetStat("MegaRoadMap::Edges", m_megaRoadmap->get_num_edges());
    Simulation::GetStatClass()->SetStat(capability+"::Vertices", graph->get_num_vertices());
    Simulation::GetStatClass()->SetStat(capability+"::Edges", graph->get_num_edges());
  }

}

WholeTask*
Coordinator::
GetWholeTask(std::shared_ptr<MPTask> _subtask){
  return m_subtaskMap[_subtask];
}
