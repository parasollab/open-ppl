#include "Coordinator.h"

#include <limits>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPLibrary/MPTools/TRPTool.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/MPHandoffTemplate.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Simulator/Simulation.h"

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
        m_handoffEnvironment = std::unique_ptr<Environment>(new Environment(_node));
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

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  GenerateDummyAgents();

  GenerateHandoffTemplates();

  GenerateRoadmaps(); 

  AssignInitialTasks();
}


void
Coordinator::
Step(const double _dt) {
  //std::cout << "Initializing step for bcg." << std::endl;
  Initialize();

  if(this->m_debug)
    std::cout << "___________________________________________________________"
              << std::endl;
  for(auto agent : m_memberAgents)  {
    HandoffAgent* childAgent = static_cast<HandoffAgent*>(agent);
    if(this->m_debug)
      std::cout << agent->GetRobot()->GetLabel()
                << " has plan: "
                << childAgent->HasPlan()
                << std::endl
                << "Has task: "
                << childAgent->GetTask().get()
                << std::endl;
  }

  ArbitrateCollision();

  for(auto agent : m_memberAgents){
    agent->Step(_dt); 
  }

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

  m_solution = nullptr;
  m_library  = nullptr;
}

/*-------------------------- Coordinator Interface ---------------------------*/

void
Coordinator::
AssignTask(std::shared_ptr<MPTask> _nextTask) {
  
  HandoffAgent* minAgent = nullptr;
  double minCost = std::numeric_limits<double>::max();
  
  // Generate the cost of a task for each agent
  std::vector<std::thread> costThreads(m_memberAgents.size());
  for(auto agent : m_memberAgents){    
    auto tempThread = std::thread([agent, _nextTask](){
      agent->GenerateCost(_nextTask);
    });
    costThreads.push_back(std::move(tempThread));
  }
  for(auto& thread : costThreads){
    thread.join();
  }
  // Assign the task to the agent with the lowest cost.
  for(auto agent : m_memberAgents){
    if(agent->GetPotentialCost() < minCost){
      minCost = agent->GetPotentialCost();
      minAgent = agent;
    }
  }
  //TODO: Setup tasks as a queue for agents since they could have more than one
  minAgent->SetTask(_nextTask);

  // See how long this agent will take to complete the subtask and if it
  // ends in a handoff add the next subtask to the queue
  double endTime = minAgent->GetTaskTime() + m_currentTime;
  std::shared_ptr<MPTask> newSubtask = GetNextSubtask(m_subtaskMap[_nextTask]);
  newSubtask->SetStartTime(endTime);

  AddSubtask(newSubtask);

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
      //std::cout << "Collision detected." << std::endl;
       needReplan.push_back(std::make_pair(childAgent, group));
       
       if(this->m_debug)
         std::cout << childAgent->GetRobot()->GetLabel() << " is in collision"
                   << std::endl;
       childAgent->ClearPlan();
    }
  }
  for(auto pair : needReplan){
    HandoffAgent* groupAgent = static_cast<HandoffAgent*>(pair.first);
    //std::cout << groupAgent->GetRobot()->GetLabel() << std::endl;
    vector<Agent*> group = pair.second;
    if(IsHighestPriority(groupAgent, group)){
      if(this->m_debug)
        std::cout << "Updating Version Map" << std::endl;
      UpdateVersionMap(groupAgent, group);
    }
    else{
      groupAgent->PauseAgent(5);
    }
  }
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
  //std::cout << "Checking for highest priority" << std:: endl;
  //std::cout << "Size of group: " << _group.size() << std::endl;
  size_t maxPriority = 0;
  for(auto agent : _group){
    size_t currentPriority = GetPriority(agent);
    //std::cout << "Current Priority for" << agent->GetRobot()->GetLabel()
    //          << ": " << currentPriority << std::endl;
    if(currentPriority >= maxPriority)
      maxPriority = currentPriority;
  }
  return GetPriority(_a) > maxPriority;
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

  task->SetStarted();
  // Set the member's current task.
  _member->SetTask(task);

  // Add the task to the MPProblem.
  //m_robot->GetMPProblem()->AddTask(std::move(task));
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


void 
Coordinator::
AddSubtask(std::shared_ptr<MPTask> _subtask) {
  for(auto it = m_unassignedTasks.begin(); it != m_unassignedTasks.end(); it++){
    if(_subtask->GetStartTime() < it->get()->GetStartTime()){
      m_unassignedTasks.insert(it, _subtask);
    }
  }
}


double
Coordinator::
GetCurrentTime(){
  return m_currentTime;
}

/*--------------------------- Initialize Helpers ------------------------------*/

void 
Coordinator::
GenerateHandoffTemplates(){
  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task. 
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_handoffEnvironment));
  
  // Loop through all handoff templates
  for(auto currentTemplate : m_robot->GetMPProblem()->GetHandoffTemplates()){
    //TODO:: Clear current dynamic obsalces (other paths) from problem
    auto handoffTasks = currentTemplate->GetTasks();
    for(auto task : handoffTasks){
      // TODO: Come up with a way to create start constraints for the handoffs
      // (currently preset in XML)
      std::string taskCapability = task->GetCapability();
      Agent* dummyAgent = m_dummyMap[taskCapability];
      Robot* dummyRobot = problemCopy->GetRobot(dummyAgent->GetRobot()->GetLabel()); 
      task->SetRobot(dummyRobot);
      std::unique_ptr<MPSolution> handoffSolution = std::unique_ptr<MPSolution>(new MPSolution(dummyRobot));
      //TODO:: Solve using safe interval path planning method
      m_library->Solve(problemCopy.get(), task.get(), handoffSolution.get());
      // Store the path for each task in the handoff
      currentTemplate->AddRoadmapGraph(*handoffSolution->GetRoadmap()->GetGraph());
      //TODO:: Store path and robot as dynamic obstacle
    }
  }
}

void
Coordinator::
GenerateDummyAgents(){
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : m_memberAgents){
    std::string capability = agent->GetAgentCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      auto robot = agent->GetRobot();
      HandoffAgent* dummyAgent = new HandoffAgent(robot);
      // Set parent agent to itself, so that it plans using the proper capability.
      dummyAgent->SetParentAgent(dummyAgent);
      m_dummyMap[capability] = dummyAgent;
    }
    agent->SetParentAgent(m_dummyMap[capability]);
  }
}

void
Coordinator::
GenerateRoadmaps(){
  // Initialize the roadmap for each agent capability.
  std::vector<std::thread> roadmapThreads(m_memberAgents.size());
  for(auto const& element : m_dummyMap){
    HandoffAgent* agent = element.second;
    auto task = m_robot->GetMPProblem()->GetTasks(m_robot).front();
    agent->SetTask(task);
    auto tempThread = std::thread([agent](){
      agent->InitializeRoadmap();
    });
    roadmapThreads.push_back(std::move(tempThread));
  }
  for(auto& thread : roadmapThreads){
    thread.join();
  }
}

void 
Coordinator::
AssignInitialTasks(){
  // Load m_unassignedTasks with the initial subtasks for all tasks. 
  for(auto wholeTask : m_wholeTasks){
    auto subtask = GetNextSubtask(wholeTask);
    if(subtask)
      m_unassignedTasks.push_back(subtask);
  }

  // Assign all of the tasks (and their subtasks) to different agents.
  while(!m_unassignedTasks.empty()){
    std::shared_ptr<MPTask> nextTask = m_unassignedTasks.front();
    AssignTask(nextTask);
    m_unassignedTasks.pop_front();
  }
}
