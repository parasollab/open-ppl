#include "Coordinator.h"

#include <limits>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Behaviors/Agents/HandoffAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPLibrary/MPTools/TRPTool.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/MPHandoffTemplate.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Simulator/Simulation.h"
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
  std::cout << "Done Initializing Agents" << std::endl;

  GenerateDummyAgents();
  std::cout << "Done Generating Dummy Agents" << std::endl;

  GenerateHandoffTemplates();
  std::cout << "Done Generating Handoff Templates" << std::endl;

  m_megaRoadmap = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(m_robot);
  auto task = m_library->GetMPProblem()->GetTasks(m_robot).front();
  m_library->SetTask(task.get());
  TranslateHandoffTemplates();
  std::cout << "Done Translating Handoff Templates" << std::endl;

  Roadmap<MPTraits<Cfg>> testRm(m_robot);
  testRm.SetGraph(m_megaRoadmap);
  testRm.Write("postTranslate.map", m_robot->GetMPProblem()->GetEnvironment());

  SetupWholeTasks();
  std::cout << "Done Setting up Whole Tasks" << std::endl;

  testRm.SetGraph(m_megaRoadmap);
  testRm.Write("postSetupWholeTask.map", m_robot->GetMPProblem()->GetEnvironment());

  GenerateRoadmaps();
  std::cout << "Done Generating Roadmaps" << std::endl;

  testRm.SetGraph(m_megaRoadmap);
  testRm.Write("postGenerateRoadaps.map", m_robot->GetMPProblem()->GetEnvironment());

  PlanWholeTasks();
  std::cout << "Done Planning Whole Tasks" << std::endl;

  testRm.SetGraph(m_megaRoadmap);
  testRm.Write("postPlanWholeTask.map", m_robot->GetMPProblem()->GetEnvironment());

  CopyCapabilityRoadmaps();
  std::cout << "Done Copying Capability Roadmaps" << std::endl;

  AssignInitialTasks();
  std::cout << "Done Assigning Initial Tasks" << std::endl;
  testRm.SetGraph(nullptr);
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
    if(this->m_debug)
      std::cout << agent->GetRobot()->GetLabel()
                << " has plan: "
                << agent->HasPlan()
                << std::endl
                << "Has task: "
                << agent->GetTask().get()
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
  delete m_megaRoadmap;

  m_solution = nullptr;
  m_library  = nullptr;
  m_megaRoadmap = nullptr;

  for(auto task : m_wholeTasks){
    delete task;
  }
}

/*-------------------------- Coordinator Interface ---------------------------*/

void
Coordinator::
AssignTask(std::shared_ptr<MPTask> _nextTask) {

  HandoffAgent* minAgent = nullptr;
  double minCost = std::numeric_limits<double>::max();

  // Generate the cost of a task for each agent
  // TODO: Thread this. Our attempt was the commented code below
  //std::vector<std::thread> costThreads(m_memberAgents.size());
  for(auto agent : m_memberAgents){
    agent->GenerateCost(_nextTask);
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
  std::cout << "MinAgent: " << minAgent->GetRobot()->GetLabel() << std::endl;
  // Add subtask to agent queue of subtasks
  minAgent->AddSubtask(_nextTask);

  // See how long this agent will take to complete the subtask and if it
  // ends in a handoff add the next subtask to the queue
  double endTime = minAgent->GetTaskTime() + m_currentTime;
  std::shared_ptr<MPTask> newSubtask = GetNextSubtask(m_subtaskMap[_nextTask]);
  if(newSubtask){
    m_subtaskMap[newSubtask] = m_subtaskMap[_nextTask];
    newSubtask->SetStartTime(endTime);

    AddSubtask(newSubtask);
  }

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
  if(m_unassignedTasks.empty()){
    m_unassignedTasks.push_back(_subtask);
    return;
  }
  for(auto it = m_unassignedTasks.begin(); it != m_unassignedTasks.end(); it++){
    if(_subtask->GetStartTime() < it->get()->GetStartTime()){
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

/*--------------------------- Initialize Functions ------------------------------*/

void
Coordinator::
GenerateDummyAgents(){
  // Load the dummyMap, which stores a dummy agent for each agent capability.
  for(auto agent : m_memberAgents){
    std::string capability = agent->GetCapability();
    if(m_dummyMap.find(capability) == m_dummyMap.end()){
      //auto robot = agent->GetRobot();
      //HandoffAgent* dummyAgent = new HandoffAgent(robot);
      //dummyAgent->SetCapability(capability);
      // Set parent agent to itself, so that it plans using the proper capability.
      //dummyAgent->SetParentAgent(dummyAgent);
      m_dummyMap[capability] = agent;
    }
    //agent->SetParentAgent(m_dummyMap[capability]);
  }
}

void
Coordinator::
GenerateHandoffTemplates(){
  // Loop through handoff templates, set start constraints for handoff, set
  // dummy robot for handoff task by capability, and solve handoff task.
  auto originalProblem = m_robot->GetMPProblem();
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  problemCopy->SetEnvironment(std::move(m_handoffEnvironment));
  m_library->SetMPProblem(problemCopy.get());
  // Set robots to virtual so that planning handoffs does not cause collisions
  std::list<HandoffAgent*> unusedAgents;

  // Loop through all handoff templates
  for(auto currentTemplate : m_robot->GetMPProblem()->GetHandoffTemplates()){
    //currentTemplate->SetHandoffPolyhedron(m_handoffEnvironment->GetBoundary());
    unusedAgents.clear();
    std::copy(m_memberAgents.begin(), m_memberAgents.end(), std::back_inserter(unusedAgents));
    auto handoffTasks = currentTemplate->GetTasks();
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
          auto sampler = m_library->GetSampler("UniformRandomFree");
          size_t numNodes = 1, numAttempts = 100;
          sampler->Sample(numNodes, numAttempts, boundingBox,
              std::back_inserter(goalPoints));

          if(goalPoints.empty())
            throw RunTimeException(WHERE, "No valid final handoff position for the robot.");

          goalPoints[0].ConfigureRobot();
          break;
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
      m_library->Solve(problemCopy.get(), task.get(), handoffSolution.get());
      taskRobot->GetMultiBody()->Configure(currentConfig);

      std::cout << "Size of path " << handoffSolution->GetPath()->Cfgs().size() << std::endl;


      Roadmap<MPTraits<Cfg>> testRm(m_robot);
      testRm.SetGraph(handoffSolution->GetRoadmap()->GetGraph());
      testRm.Write("indHandoffTemplate" + std::to_string(check) + ".map", problemCopy->GetEnvironment());
      testRm.SetGraph(nullptr);
      check++;

      // Store the roadmap for each task in the handoff
      currentTemplate->AddRoadmapGraph(handoffSolution->GetRoadmap()->GetGraph());

      // Add final configuration of path to template
      std::cout << "Path: " << handoffSolution->GetPath()->Size() << std::endl;
      currentTemplate->AddHandoffCfg(handoffSolution->GetPath()->Cfgs().back(), originalProblem);
      std::cout << "Handoff Cfg: " << handoffSolution->GetPath()->Cfgs().back() << std::endl;
    }


    currentTemplate->ConnectRoadmaps(m_robot, originalProblem);

    std::cout << "Trying to write handoffTemplate Map" << std::endl;
    Roadmap<MPTraits<Cfg>> testRm(m_robot);
    testRm.SetGraph(currentTemplate->GetConnectedRoadmap());
    testRm.Write("handoffTemplate.map", problemCopy->GetEnvironment());
    testRm.SetGraph(nullptr);

    // Reset the agents to non-virtual, since they could be used in the next
    // template.
    for(auto agent : unusedAgents){
      problemCopy->GetRobot(agent->GetRobot()->GetLabel())->SetVirtual(false);
    }
  }

  /*for(auto temp : m_robot->GetMPProblem()->GetHandoffTemplates()){
    std::cout << "There is a handoff template" << std::endl;
    //temp->ConnectRoadmaps(m_robot, m_robot->GetMPProblem());
    auto g = temp->GetConnectedRoadmap();
    for(auto vit = g->begin(); vit!=g->end(); vit++){
      std::cout << "There is a vit" << std::endl;
      std::cout << vit->property().PrettyPrint() << std::endl;
    }
  }*/

  m_library->SetMPProblem(originalProblem);
  m_library->SetMPSolution(m_solution);
}

void
Coordinator::
TranslateHandoffTemplates() {
  for(auto currentTemplate : m_robot->GetMPProblem()->GetHandoffTemplates()){

    if(m_debug){
      //currentTemplate->ConnectRoadmaps(m_robot, m_robot->GetMPProblem());
      auto g = currentTemplate->GetConnectedRoadmap();
      std::cout << "Original handoff position" << std::endl;
      for(auto vit = g->begin(); vit!=g->end(); vit++){
        std::cout << vit->property().PrettyPrint() << std::endl;
      }
    }


    size_t maxAttempts = currentTemplate->GetMaxAttempts();

    size_t numNodes = 1, numAttempts = 100;
    auto sampler = m_library->GetSampler("UniformRandomFree");
    auto envBoundary = m_library->GetMPProblem()->GetEnvironment()->GetBoundary();

    for(size_t attemptNum = 0; attemptNum < maxAttempts; attemptNum++){
      std::vector<Cfg> potentialPoints;
      sampler->Sample(numNodes, numAttempts, envBoundary, std::back_inserter(potentialPoints));
      if(potentialPoints.empty())
        continue;

      Cfg centerCfg = potentialPoints[0];
      RoadmapGraph<Cfg, DefaultWeight<Cfg>>* graph = currentTemplate->GetConnectedRoadmap();

      // Copy vertices and map the change in VIDs.
      std::unordered_map<VID, VID> oldToNew;
      //std::cout << "Adding vertices in Translate Templates" << std::endl;
      for(auto vit = graph->begin(); vit != graph->end(); ++vit) {
        std::cout << "Will said print here" << std::endl;
        const VID oldVID = vit->descriptor();
        auto relativeCfg = vit->property();
        TranslateCfg(centerCfg, relativeCfg);
        const VID newVID = m_megaRoadmap->AddVertex(relativeCfg);
        oldToNew[oldVID] = newVID;
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
              TranslateCfg(centerCfg, cfg);
            }
            m_megaRoadmap->AddEdge(source, target, eit->property());
          }
        }
      }
    }
  }
}

void
Coordinator::
SetupWholeTasks(){
  for(auto task : m_robot->GetMPProblem()->GetTasks(m_robot)){
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

    wholeTask->m_startPoints["coordinator"] = {startPoints[0]};
    wholeTask->m_goalPoints["coordinator"] = {goalPoints[0]};

    // Loop through each type of capability then push start/goal constraints
    // into vectors in WholeTask
    for(auto const& elem : m_dummyMap) {
      // Set library robot to the corresponding capability
      task->SetRobot(elem.second->GetRobot());
      m_library->SetTask(task.get());
      // Sample to find valid start and goal points in the environment
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

      wholeTask->m_startPoints[elem.second->GetCapability()].push_back(startPoints[0]);
      wholeTask->m_goalPoints[elem.second->GetCapability()].push_back(goalPoints[0]);
    }

    task->SetRobot(m_robot);
    m_library->SetTask(task.get());

    // Create 0 weight edges between each capability and the coordinator
    // configuration.
    //std::cout << "Adding vertices in SetupWholeTasks" << std::endl;
    auto coordinatorStartVID = m_megaRoadmap->AddVertex(wholeTask->m_startPoints["coordinator"][0]);
    wholeTask->m_startVIDs["coordinator"] = {coordinatorStartVID};
    auto coordinatorGoalVID = m_megaRoadmap->AddVertex(wholeTask->m_goalPoints["coordinator"][0]);
    wholeTask->m_goalVIDs["coordinator"] = {coordinatorGoalVID};
    const DefaultWeight<Cfg> weight;
    /*
    for(size_t i = 1; i < wholeTask->m_startPoints.size(); ++i) {
      auto agentStartVID = m_megaRoadmap->AddVertex(wholeTask->m_startPoints[i]);
      auto agentGoalVID = m_megaRoadmap->AddVertex(wholeTask->m_goalPoints[i]);
      m_megaRoadmap->AddEdge(coordinatorStartVID, agentStartVID, weight);
      m_megaRoadmap->AddEdge(coordinatorGoalVID, agentGoalVID, weight);
    }*/
    for(auto const& elem : wholeTask->m_startPoints){
      if(elem.first == "coordinator")
        continue;

    //std::cout << "Adding vertices in SetupWholeTasks again" << std::endl;
      for(auto start : elem.second) {
        auto agentStartVID = m_megaRoadmap->AddVertex(start);

        // Add the start points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentStartVID});
        wholeTask->m_startVIDs[elem.first].push_back(agentStartVID);
        m_megaRoadmap->AddEdge(coordinatorStartVID, agentStartVID, weight);
      }

    }

    for(auto const& elem : wholeTask->m_goalPoints){
      if(elem.first == "coordinator")
        continue;

      //std::cout << "Adding vertices in SetupWholeTasks once more" << std::endl;
      for(auto goal : elem.second) {
        auto agentGoalVID = m_megaRoadmap->AddVertex(goal);

        // Add the end points as in the same containter as the transformed
        // roadmaps so that it is connected to the rest of the transformed
        // roadmaps
        m_wholeTaskStartEndPoints.push_back({agentGoalVID});
        wholeTask->m_goalVIDs[elem.first].push_back(agentGoalVID);
        m_megaRoadmap->AddEdge(coordinatorGoalVID, agentGoalVID, weight);
      }

    }
    m_wholeTasks.push_back(wholeTask);
  }
  m_library->SetTask(m_robot->GetMPProblem()->GetTasks(m_robot)[0].get());
}


void
Coordinator::
GenerateRoadmaps() {
  for(auto& r : m_robot->GetMPProblem()->GetRobots()){
    r->SetVirtual(true);
  }
  for(auto const& elem : m_dummyMap){
    auto capability = elem.first;
    HandoffAgent* dummyAgent = elem.second;
    dummyAgent->InitializeRoadmap();

    std::unordered_map<size_t, size_t> handoffVIDMap;
    std::unordered_map<size_t, size_t> inverseVIDMap;
    auto graph = dummyAgent->GetMPSolution()->GetRoadmap()->GetGraph();



    // Copy handoffs of same capability into corresponding capability map
    //std::cout << "Adding vertices in GenerateRoadmaps the first time" << std::endl;
    for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
      auto roadmap = m_transformedRoadmaps[i];
      //std::cout << roadmap[0] << std::endl;
      //std::cout << roadmap.size() << std::endl;
      if(capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
          GetAgent()->GetCapability()){
        //std::cout << "Inside the if statement" << std::endl;
        for(size_t j = 0; j < roadmap.size(); j++){
          auto vid = roadmap[j];
          const auto newVID = graph->AddVertex(m_megaRoadmap->GetVertex(vid));
          handoffVIDMap[newVID] = vid;
          inverseVIDMap[vid] = newVID;
        }
      }
    }
    //std::cout << "Adding edges" << std::endl;
    for(auto vit = graph->begin(); vit != graph->end(); vit++){
      auto vid1 = vit->descriptor();
      auto orig1 = handoffVIDMap[vid1];
      //std::cout << "VID1: " << vid1 << std::endl;
      for(auto vit2 = graph->begin(); vit2 != graph->end(); vit2++){
        if(vit == vit2) continue;
        auto vid2 = vit2->descriptor();
        auto orig2 = handoffVIDMap[vid2];
        //std::cout << "VID2: " << vid2 << std::endl;
        if(m_megaRoadmap->IsEdge(orig1, orig2)){
          //std::cout << "Trying to add an edge from " << vid1
          //  << " to " << vid2 << std::endl;
          //std::cout << "Orginally " << orig1
          //  << " to " << orig2 << std::endl;
          auto edge = m_megaRoadmap->GetEdge(orig1, orig2);
          graph->AddEdge(vid1, vid2, edge);
          //std::cout << "Added an edge" << std::endl;
        }

      }
    }


    std::cout << "Done initializing capability map with corresponding cfgs from megaRoadmap"
              << std::endl;

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

    std::cout << "Done connecting handoff templates of same capability" << std::endl;

    //ConnectWholeTasks(capability, dummyAgent);

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



    //Used to see if graph is being assigned somewhere else and going out of
    //scope
    //TODO: find out of scope problem and fix it
    /*RoadmapGraph<Cfg, DefaultWeight<Cfg>>* g =
      new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(dummyAgent->GetRobot());
    *g = *graph;
    */m_capabilityRoadmaps[capability] = graph;

    // Copy over newly found vertices
    for(auto vit = graph->begin(); vit != graph->end(); ++vit){
      // Add vertices found in building the capability map to the mega roadmap
      // without copying over the handoff vertices that already exist
      //std::cout << "Adding vertices in GenerateRoadmaps" << std::endl;
      if(handoffVIDMap.find(vit->descriptor()) == handoffVIDMap.end()){
        auto megaVID = m_megaRoadmap->AddVertex(vit->property());
        handoffVIDMap[vit->descriptor()] = megaVID;
      }
    }

    std::cout << "Done copying over vertices" << std::endl;

    // Copy over newly found edges
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

    std::cout << "Done copying over edges" << std::endl;
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


  m_solution->GetRoadmap(m_robot)->SetGraph(m_megaRoadmap);

  for(auto wholeTask: m_wholeTasks){
    wholeTask->m_task->SetRobot(m_robot);
    m_library->SetTask(wholeTask->m_task.get());
    m_library->Solve(m_robot->GetMPProblem(), wholeTask->m_task.get(), m_solution, "LazyPRM",
      LRand(), "LazyCollisionAvoidance");


    wholeTask->m_wholePath = m_solution->GetPath()->Cfgs();
    std::cout << "Printing out path of whole task" << std::endl;
    for(auto cfg : wholeTask->m_wholePath){
      std::cout << cfg.PrettyPrint() << std::endl;
    }
  }


   for(auto wholeTask : m_wholeTasks){
    // Loop through path to find cfgs where robot pointer changes
    // When the robot pointer changes, create a new subtask
    // TODO: Maybe check based on capability rather than robot pointer
    Robot* checkRobot = nullptr;
    Cfg subtaskStart;
    Cfg subtaskEnd;
    // First cfg in the path is the coordinator configurations at the
    // start of the whole task
    for(size_t i = 1; i < wholeTask->m_wholePath.size()-1; i++){
      auto cfg = wholeTask->m_wholePath[i];
      std::cout << "Robot pointer in path: " << cfg.GetRobot() << std::endl;
      std::cout << cfg.PrettyPrint() << std::endl;
      // If it has not reached the handoff or end of path, keep moving end point
      // back
      if(cfg.GetRobot()==checkRobot and i != (wholeTask->m_wholePath.size()-2)){
        std::cout << "Same robot ptr" << std::endl;
        subtaskEnd = cfg;
      }
      else{
        std::cout << "Different robot ptr or end of path" << std::endl;
        // If it's the start of a new subtask, set new start point
        if(!checkRobot){
          subtaskEnd = cfg;
          checkRobot = cfg.GetRobot();
          if(i == 1){
            subtaskStart = cfg;
          }
        }
        // If we have reached a handoff or end of the path, create a subtask of
        // the wholetask
        else {
          std::cout << "End of subtask" << std::endl;
          std::shared_ptr<MPTask> subtask = std::shared_ptr<MPTask>(new MPTask(m_robot));
          // Make start and goal constrants from start and end cfgs

          if(subtaskStart == subtaskEnd){
            std::cout << "start and end are same" << std::endl;
          }

          //std::cout << "====ADDING BACK TASK WITH SAME START/GOAL====" << std::endl;
          auto startConstraint = std::unique_ptr<CSpaceConstraint>
            (new CSpaceConstraint(subtaskStart.GetRobot(), subtaskStart));
          subtask->SetStartConstraint(std::move(startConstraint));

          auto goalConstraint = std::unique_ptr<CSpaceConstraint>
            (new CSpaceConstraint(subtaskEnd.GetRobot(), subtaskEnd));
          subtask->AddGoalConstraint(std::move(goalConstraint));

          // Push task back into whole task object
          wholeTask->m_subtasks.push_back(subtask);
          checkRobot = nullptr;
          std::cout << "Start of new subtask" << std::endl;
          subtaskStart = cfg;
        }
      }
    }
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
    agent->SetRoadmapGraph(graph);

    auto g = new RoadmapGraph<Cfg, DefaultWeight<Cfg>>(agent->GetRobot());
    *g = *graph;
    Roadmap<MPTraits<Cfg>> testRm(agent->GetRobot());
    testRm.SetGraph(g);
    testRm.Write(agent->GetRobot()->GetLabel() + ".map", m_robot->GetMPProblem()->GetEnvironment());
  }
}

void
Coordinator::
AssignInitialTasks() {
  // Load m_unassignedTasks with the initial subtasks for all tasks.
  for(auto wholeTask : m_wholeTasks){
    auto subtask = GetNextSubtask(wholeTask);
    if(subtask){
      m_unassignedTasks.push_back(subtask);
      m_subtaskMap[subtask] = wholeTask;
    }
  }

  // Assign all of the tasks (and their subtasks) to different agents.
  int numSubTs = 0;
  while(!m_unassignedTasks.empty()){
    std::cout << "SubTask: " << numSubTs << std::endl;
    numSubTs++;
    std::shared_ptr<MPTask> nextTask = m_unassignedTasks.front();
    AssignTask(nextTask);
    m_unassignedTasks.pop_front();
  }
}

/*--------------------------- Helpers ------------------------------*/

void
Coordinator::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2];

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
  for(auto vid1 : _roadmap1) {
    Cfg cfg1 = m_megaRoadmap->GetVertex(vid1);
    auto cfg1Copy = cfg1;
    cfg1Copy.SetRobot(dummyRobot);
    if(!m_library->GetValidityChecker("pqp_solid")->IsValid(cfg1Copy, "cfg1"))
      continue;

    for(auto vid2 : _roadmap2) {
      Cfg cfg2 = m_megaRoadmap->GetVertex(vid2);
      auto cfg2Copy = cfg2;
      cfg2Copy.SetRobot(dummyRobot);
      if(vid1 == vid2 || cfg1Copy == cfg2Copy) continue;
      if(!m_library->GetValidityChecker("pqp_solid")->IsValid(cfg2Copy, "cfg2"))
        continue;


      // Attempt to plan between the two roadmaps
      MPTask* tempTask = new MPTask(dummyRobot);
      std::unique_ptr<CSpaceConstraint> start(new CSpaceConstraint(dummyRobot, cfg1Copy));
      std::unique_ptr<CSpaceConstraint> goal(new CSpaceConstraint(dummyRobot, cfg2Copy));
      tempTask->SetStartConstraint(std::move(start));
      tempTask->AddGoalConstraint(std::move(goal));
      std::cout << "Calling solve" << std::endl;
      m_library->Solve(problemCopy.get(), tempTask, _agent->GetMPSolution());
      std::cout << "Finish solving" << std::endl;

      if(!_agent->GetMPSolution()->GetPath()->Cfgs().empty()) {
        std::cout << "Finishing connecting distinct roadmaps" << std::endl;
        m_library->SetMPProblem(originalProblem);
        return;
      }
    }
  }
  m_library->SetMPProblem(originalProblem);
}

void
Coordinator::
ConnectWholeTasks(std::string _capability, HandoffAgent* _agent){
  std::cout << "Connecting Whole Tasks" << std::endl;
  for(size_t i = 0; i < m_transformedRoadmaps.size(); i++){
    auto roadmap = m_transformedRoadmaps[i];
    if(_capability == m_megaRoadmap->GetVertex(roadmap[0]).GetRobot()->
        GetAgent()->GetCapability()){
      for(auto wholeTask : m_wholeTasks){
        for(auto vid : wholeTask->m_startVIDs[_capability]){
          ConnectDistinctRoadmaps(roadmap, {vid}, _agent);
          //wholeTask->m_wholePath = _agent->GetMPSolution()->GetPath()->Cfgs();
          std::cout << "Printing out edges to wholeTask start points" << std::endl;
          for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
            if(vit->descriptor() == vid){
              std::cout << "Found start vit" << std::endl;
              for(auto eit = vit->begin(); eit != vit->end(); eit++){
                std::cout << "edge between " << eit->source() << " and " << eit->target() << std::endl;
              }
            }
          }
        }
        for(auto vid : wholeTask->m_goalVIDs[_capability]){
          ConnectDistinctRoadmaps(roadmap, {vid}, _agent);
          std::cout << "Printing out edges to wholeTask end points" << std::endl;
          for(auto vit = m_megaRoadmap->begin(); vit != m_megaRoadmap->end(); vit++){
            if(vit->descriptor() == vid){
              std::cout << "Found end vit" << std::endl;
              for(auto eit = vit->begin(); eit != vit->end(); eit++){
                std::cout << "edge between " << eit->source() << " and " << eit->target() << std::endl;
              }
            }
          }
        }

      }
      // Find point in both roadmaps and try to connect them
    }
  }
}
