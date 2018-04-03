#include "BatteryConstrainedGroup.h"

#include <limits>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "Behaviors/Agents/PathFollowingChildAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPLibrary/MPTools/TRPTool.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"

/*------------------------------ Construction --------------------------------*/

BatteryConstrainedGroup::
BatteryConstrainedGroup(Robot* const _r) : Agent(_r) {
}


BatteryConstrainedGroup::
BatteryConstrainedGroup(Robot* const _r, XMLNode& _node) : Agent(_r) {
  
  // Parse the labels of the group members.
  for(auto& child : _node) {
    // Parse the robot label.
    const std::string memberLabel = child.Read("label", true, "",
        "The label of the member robot.");
    m_memberLabels.push_back(memberLabel);

    // Parse the initial role.
    std::string role = child.Read("role", true, "", "This robot's initial "
        "role in the group {worker, helper}.");

    std::transform(role.begin(), role.end(), role.begin(), ::tolower);
    Role memberRole;
    if(role == "worker")
      memberRole = Worker;
    else if(role == "helper")
      memberRole = Helper;
    else if(role == "charging")
      memberRole = Charging;
    else
      throw ParseException(child.Where(), "Unrecognized role '" + role + "'.");

    m_initialRoles[memberLabel] = memberRole;
  }

  m_handoff = _node.Read("handoff", true, m_handoff, "Should workers wait "
      "for helpers to arrive before going back to charge?");

  // This is a coordinator agent, which does not make sense without some group
  // members to coordinate. Throw an exception if it has no members.
  if(m_memberLabels.empty())
    throw ParseException(_node.Where(), "BatteryConstrainedGroup requires at "
        "least one member robot.");
}


BatteryConstrainedGroup::
~BatteryConstrainedGroup() {
  Uninitialize();
}


std::unique_ptr<Agent> 
BatteryConstrainedGroup::
Clone(Robot* const _r) const {
  throw RunTimeException(WHERE, "Not yet implemented.");
  return {nullptr};
}
/*------------------------------ Agent Interface -----------------------------*/

void
BatteryConstrainedGroup::
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
    /// @TODO Generalize code so that this is not necessary.
    Agent* memberAgent = member->GetAgent();
    PathFollowingChildAgent* a = dynamic_cast<PathFollowingChildAgent*>(
        memberAgent);
    if(!a)
      throw RunTimeException(WHERE, "Incompatible agent type specified for "
          "group member '" + memberLabel + "'.");
    m_memberAgents.push_back(a);

    a->SetParentAgent(this);

    // Set the initial role for this member.
    SetRole(memberAgent, m_initialRoles[memberLabel]);

    // Set the initial priority.
    SetPriority(memberAgent, IsWorker(memberAgent) ? 1000 + priority++ : priority++);
  }

  // Initialize the version map.
  /// @TODO Generalize code so that this is not necessary.
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

  // Initialize the charging locations.
  InitializeChargingLocations();

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  // Generate the shared roadmap. This uses an empty task, so an appropriate map
  // evaluator should be chosen to get the desired PRM coverage.
  std::unique_ptr<MPTask> sharedRoadmapTask(new MPTask(m_robot));
  sharedRoadmapTask->SetLabel("InitializeSharedRoadmap");
  auto task = sharedRoadmapTask.get();
  problem->AddTask(std::move(sharedRoadmapTask));
  //auto task = m_robot->GetMPProblem()->GetTasks(m_robot).front();
  m_library->Solve(problem, task, m_solution);
  task->SetCompleted();

  /*std::cout << "Got to James's Test stuff" << std::endl;
  //James testing trp stuff
  std::vector<Robot*> workerRobots;
  for(auto worker : GetWorkers()){
    workerRobots.push_back(worker->GetRobot());
    worker->GetRobot()->SetVirtual(true);
  }
  auto trp = m_library->GetMPTools()->GetTRPTool("trpTool");
  trp->Initialize(m_robot, workerRobots);
  trp->Search();

  for(auto worker : GetWorkers()){
    worker->GetRobot()->SetVirtual(false);
  }
  std::cout << "Finished James's Test stuff" << std::endl;*/
}


void
BatteryConstrainedGroup::
Step(const double _dt) {
  //std::cout << "Initializing step for bcg." << std::endl;
  Initialize();

  //std::cout << "Arbitrate Collision for bcg." << std::endl;
  ArbitrateCollision();
  
  //std::cout << "Stepping agents for bcg." << std::endl;
  for(auto agent : m_memberAgents){
    //std::cout << "Battery Check for bcg:: " << agent->GetRobot()->GetLabel() << "." << std::endl;
    BatteryCheck(agent);
    //std::cout << "Stepping the agent" << std::endl;
    agent->Step(_dt);
    //Checks if GoToHelp agent has left charging location
    if(/*GetRole(agent) == Helper or */GetRole(agent) == GoingToHelp){
      for(auto& chargingLocation : m_chargingLocations){
        if(chargingLocation.second != agent)
          continue;
        if(!chargingLocation.first->InBoundary(agent->GetRobot()->
              GetDynamicsModel()->GetSimulatedState())){
          ClearChargingLocation(agent);
        }
        break;
      }
    }
  }
  //charges all agents currently at charging locations.
  for(auto& chargingLocation : m_chargingLocations){
    if(chargingLocation.second){
      chargingLocation.second->GetRobot()->GetBattery()->Charge(m_chargingRate);
    }
  }
  //TODO need to be sure that time res is the right way to do this
  m_currentTime += m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
}


void
BatteryConstrainedGroup::
SetBatteryBreak(BatteryBreak _break, Agent* _member){
  //TODO: Need to account for battery breing drained during planning.
  // Pretty sure that is taken care of by checking battery break after planning
  if(GetRole(_member)!=Worker)
    return;
  m_batteryBreaks[_member] =  _break;
}



void
BatteryConstrainedGroup::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  delete m_solution;
  delete m_library;

  m_chargingLocations.clear();
  m_solution = nullptr;
  m_library  = nullptr;
}

/*-------------------------- Coordinator Interface ---------------------------*/

void
BatteryConstrainedGroup::
AssignTask(Agent* const _member) {
  PathFollowingChildAgent* childAgent = static_cast<PathFollowingChildAgent*>(_member);
  // Assign a task based on the state of the member.
  //std::cout << _member->GetRobot()->GetLabel() << ": " << GetRole(_member) << std::endl;
  switch(GetRole(_member)){
    case Worker: 
      AssignTaskWorker(_member);
      break;
    case Helper:
      AssignTaskHelper(_member);
      break;
    case WaitingForHelp:
      AssignTaskWaiting(_member);
      break;
    case ReturningToCharge:
      std::cout << "Calling assign task for ReturningToCharge: " 
                << _member->GetRobot()->GetLabel() 
                << std::endl;
      if(IsAtChargingLocation(_member)){
        // TODO: Set task to nullptr
        SetRole(_member, Charging);
        SetPriority(_member, GetPriority(_member) - 500);
        m_helperMap.erase(m_helperMap.find(_member));
      }
      else{
        GoToCharge(_member);
      }
      break;
    case Charging:
      // TODO: update battery level
      if(childAgent->IsBatteryHigh())
        SetRole(childAgent, Helper);
      break;
    case GoingToHelp:
      CheckReachedWorker(_member);
      break;
  }
}

// TODO: Add time horizon code / dynamic obstacle code
void
BatteryConstrainedGroup::
ArbitrateCollision() {
  std::vector<std::pair<Agent*, std::vector<Agent*>>> needReplan;
  for(auto agent : m_memberAgents) {
    PathFollowingChildAgent* childAgent = 
      static_cast<PathFollowingChildAgent*>(agent);
    if(childAgent->IsPlanning())
      continue;
    const double distanceThreshold = 3. *
      childAgent->GetRobot()->GetMultiBody()->GetBoundingSphereRadius();
    auto group = childAgent->ProximityCheck(distanceThreshold);
    if(!group.empty() and !ValidateVersionMap(childAgent, group)){
      //std::cout << "Collision detected." << std::endl;
       needReplan.push_back(std::make_pair(childAgent, group));
       // Helper reached point of collision but is within handoff proximity and
       // can switch with its assigned worker.
       /*if(GetRole(childAgent) == GoingToHelp){
         CheckReachedWorker(agent);
       }*/
       childAgent->ClearPlan(); 
       // TODO: Stop clearing task, pause agent if not highest priority
       // TODO: Determine if this agent was going to a charging location, clear
       // said charging location.
    }
  }
  for(auto pair : needReplan){
    PathFollowingChildAgent* groupAgent = static_cast<PathFollowingChildAgent*>(pair.first);
    //std::cout << groupAgent->GetRobot()->GetLabel() << std::endl;
    vector<Agent*> group = pair.second;
    if(IsHighestPriority(groupAgent, group)){
      std::cout << "Updating Version Map" << std::endl;
      UpdateVersionMap(groupAgent, group);
    }
    else{
      groupAgent->PauseAgent(1);
      //groupAgent->SetTask(nullptr);
    }
  }
}

/*--------------------------- Member Management ------------------------------*/

void
BatteryConstrainedGroup::
SetPriority(Agent* const _a, const size_t _priority) {
  std::cout << "Setting priority of: " << _priority << std::endl
            << "For: " << _a->GetRobot()->GetLabel() << std::endl;
  m_memberPriorities[_a] = _priority;
}


size_t
BatteryConstrainedGroup::
GetPriority(Agent* const _a) {
  return m_memberPriorities[_a];
}

bool
BatteryConstrainedGroup::
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
BatteryConstrainedGroup::
SetRole(Agent* const _a, const Role _r) {
  m_roleMap[_a] = _r;
}

BatteryConstrainedGroup::Role
BatteryConstrainedGroup::
GetRole(Agent* const _a) const {
  return m_roleMap.at(_a);
}

bool
BatteryConstrainedGroup::
IsWorker(Agent* const _a) const {
  return GetRole(_a) == Worker;
}

std::vector<Agent*>
BatteryConstrainedGroup::
GetWorkers() {
  std::vector<Agent*> output;

  for(auto member : m_memberAgents)
    if(GetRole(member) == Worker)
      output.push_back(member);

  return output;
}

std::vector<Agent*>
BatteryConstrainedGroup::
GetHelpers() {
  std::vector<Agent*> output;

  for(auto member : m_memberAgents)
    if(GetRole(member) == Helper)
      output.push_back(member);

  return output;
}


Agent*
BatteryConstrainedGroup::
GetNearestHelper(Agent* const _member) {
  // Get _member's current location.
  auto robot = _member->GetRobot();
  const Cfg memberPos = robot->GetDynamicsModel()->GetSimulatedState();

  // Compare with all helpers and return the nearest.
  double nearestDistance = std::numeric_limits<double>::max();
  Agent* nearestHelper = nullptr;

  auto dm = m_library->GetDistanceMetric("connectedFreeSpace");

  for(auto helper : GetHelpers()) {
    const Cfg helperPos = helper->GetRobot()->GetDynamicsModel()->GetSimulatedState();
    const double distance = dm->Distance(memberPos, helperPos);

    if(distance < nearestDistance) {
      nearestDistance = distance;
      nearestHelper = helper;
    }
  }

  return nearestHelper;
}


void
BatteryConstrainedGroup::
DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where) {
  // Create a task to send the member to the desired location. Use the
  // coordinator robot because this is shared-roadmap planning.
  std::shared_ptr<MPTask> task(new MPTask(m_robot));
  std::unique_ptr<BoundaryConstraint> destination(
      new BoundaryConstraint(m_robot, std::move(_where))
  );
  
  
  task->AddGoalConstraint(std::move(destination));
  //TODO put back to m_debug
  if(true){
    std::cout << "SENDING " << _member->GetRobot()->GetLabel() << " TO:" << std::endl;
    for(const auto& constraint : task->GetGoalConstraints())
      std::cout << "\t" << *(constraint->GetBoundary()) << std::endl;
  }
  
  task->SetStarted();
  // Set the member's current task.
  _member->SetTask(task);

  // Add the task to the MPProblem.
  //m_robot->GetMPProblem()->AddTask(std::move(task));
}


bool
BatteryConstrainedGroup::
InHandoffProximity(Agent* const _member1, Agent* const _member2) {
  auto robot1 = _member1->GetRobot(),
       robot2 = _member2->GetRobot();

  auto cfg1 = robot1->GetDynamicsModel()->GetSimulatedState(),
       cfg2 = robot2->GetDynamicsModel()->GetSimulatedState();

  /// @TODO: Parse the distance metric string instead of hard coding.
  auto dm = m_library->GetDistanceMetric("positionEuclidean");

  const double distance = dm->Distance(cfg1, cfg2);

  // Use twice the average diameter as the handoff distance.
  const double threshold = 5. *
    (robot1->GetMultiBody()->GetBoundingSphereRadius() +
     robot2->GetMultiBody()->GetBoundingSphereRadius());
  //std::cout << "Distance from worker/helper: " << distance 
            //<< " threshold: " << threshold << std::endl;
  return distance < threshold;
}

void
BatteryConstrainedGroup::
UpdateVersionMap(Agent* const _member, std::vector<Agent*> _agents) {
  for(Agent* agent : _agents){
    PlanningAgent* planningAgent = static_cast<PlanningAgent*>(agent);
    // Update each proximity agent's knowledge about this member's plan version.
    m_versionMap[_member][planningAgent] = planningAgent->GetPlanVersion(); 
  }
}

bool
BatteryConstrainedGroup::
ValidateVersionMap(Agent* const _member, std::vector<Agent*> _agents) {
  for(Agent* agent : _agents){
    PlanningAgent* planningAgent = static_cast<PlanningAgent*>(agent);
    /*std::cout << "Actual Version Number for "
                  << agent->GetRobot()->GetLabel() << ": "
                  << planningAgent->GetPlanVersion() << std::endl
                  << _member->GetRobot()->GetLabel() << " stored version for "
                  << agent->GetRobot()->GetLabel() << ": "
                  << m_versionMap[_member][planningAgent]
                  << std::endl;
    */
    if(m_versionMap[_member][planningAgent] != planningAgent->GetPlanVersion())
      return false;
  }
  return true;
}


void
BatteryConstrainedGroup::
CheckReachedWorker(Agent* const _member){
  std::cout << "Checking if the helper has reach the worker" << std::endl;
  std::cout << "Looking for helper: " << _member << std::endl;
  for(auto agentPair : m_helperMap){
    //find the worker the helper is assigned to
    std::cout << "Worker: " << agentPair.first << std::endl;
    std::cout << "Helper: " << agentPair.second << std::endl;
    if(agentPair.second != _member)
      continue;
    //worker is not ready to swap
    std::cout << "Worker Role: " << GetRole(agentPair.first) << std::endl;
    if(GetRole(agentPair.first) == Worker)
      break;
    if(InHandoffProximity(agentPair.first, _member)){
      PerformHelperSwap(_member, agentPair.first);
    }
  }
}


/*---------------------------- Charging Locations ----------------------------*/

void
BatteryConstrainedGroup::
InitializeChargingLocations() {
  std::cout << "Initializing charging locations at: " << std::endl;
  for(Agent* const agent : m_memberAgents) {
    // Skip agents which are neither helpers nor charging.
    if(GetRole(agent) != Helper and GetRole(agent) != Charging)
      continue;

    // Get the position of the member.
    auto position = agent->GetRobot()->GetDynamicsModel()->GetSimulatedState().GetPosition();

    // Create a new charging boundary using this position.
    std::unique_ptr<CSpaceBoundingSphere> boundary(new CSpaceBoundingSphere(position, .2));
    //boundary->ShrinkToPoint(position);
    m_chargingLocations.emplace_back(std::move(boundary), agent);
    std::cout << position << std::endl;
  }
}


std::pair<Boundary*, double>
BatteryConstrainedGroup::
FindNearestChargingLocation(Agent* const _a) {
  auto robot = _a->GetRobot();
  auto currentPos = robot->GetDynamicsModel()->GetSimulatedState();
  //std::cout << "HERE1" << std::endl;
  Boundary* bestChargingLocation = nullptr;
  double bestDistance = std::numeric_limits<double>::max();

  /// @TODO Parse the distance metric from the agent XML node.
  auto dm = m_library->GetDistanceMetric("connectedFreeSpace");

  std::pair<std::unique_ptr<Boundary>, Agent*>* goalLocation = nullptr;

  // Find the closest charging location to m_robot
  //std::cout << "Printing agents at charging locations" << std::endl;
  for(auto& chargingLocation : m_chargingLocations) {
    //std::cout << chargingLocation.second->GetRobot()->GetLabel() << std::endl;
    // Skip occupied locations.
    if(chargingLocation.second && chargingLocation.second != _a)
      continue;
    // Get a configuration in the charging region.
    Boundary* region = chargingLocation.first.get();
    Cfg chargingCfg(robot);
    chargingCfg.GetRandomCfg(region);

    // Get the freespace distance.
    const double distance = dm->Distance(currentPos, chargingCfg);
    //std::cout << "Distance: " << distance << std::endl;
    // If this region is closer, save it as the best choice.
    if(distance < bestDistance) {
      bestDistance = distance;
      bestChargingLocation = region;
      goalLocation = &chargingLocation;
    }
    //std::cout << "Best Distance: " << bestDistance << std::endl;
  }
  if(bestChargingLocation->
      InBoundary(m_helperMap[_a]->GetRobot()->GetDynamicsModel()->GetSimulatedState())){
    std::cout << "My helper is on the charging location." << std::endl;

  }
  //If there is no empty charging location return an empty pointer
  if(!goalLocation or bestChargingLocation->
      InBoundary(m_helperMap[_a]->GetRobot()->GetDynamicsModel()->GetSimulatedState()))
    return {nullptr,std::numeric_limits<double>::infinity()};
  //goalLocation->second = _a;
  
  return {bestChargingLocation, bestDistance};
}

bool
BatteryConstrainedGroup::
IsAtChargingLocation(Agent* const _member) {
  std::cout << "Checking if at charging location" << std::endl;
  auto nearest = FindNearestChargingLocation(_member);
  Boundary* const location = nearest.first;
  const double distance = nearest.second;

 // We are not at a charging location if no unoccupied one was found.
 /// @TODO This will fail if _member is already at a charging location.
  if(!location)
    return false;

  

  // Define a distance threshold. We say a member is 'at' a charging location if
  // it is within this ammount.
  static constexpr double threshold = .15;
  std::cout << "Distance from charging location: " 
            << distance 
            << std::endl
            << "Threshold: "
            << threshold
            << std::endl;
  
  for(auto& charger : m_chargingLocations){
    if(charger.first.get() == location){
      // If an agent is at the charging location, assign it to the charging
      // location.
      if(location->InBoundary(_member->GetRobot()->GetDynamicsModel()->GetSimulatedState())){
        charger.second = _member;
        return true;
      }
      break;
    }
  }
  
  // If an agent is at the charging location, assign it to the charging
  // location.
  //if(distance <= threshold) {

  return false;
}

void
BatteryConstrainedGroup::
ClearChargingLocation(Agent* const _member) {
  std::cout << "Clearing charging location of: " 
            << _member->GetRobot()->GetLabel() 
            << std::endl;
  for(auto& chargingLocation : m_chargingLocations) {
    Agent* temp = chargingLocation.second;
    if(temp == _member)
      chargingLocation.second = nullptr;
  }
}

void
BatteryConstrainedGroup::
GoToCharge(Agent* const _member) {
  std::cout << _member->GetRobot()->GetLabel() 
            << "Calling find nearest charging location" 
            << std::endl;
  // Find the nearest charging location.
  auto nearest = FindNearestChargingLocation(_member);

  // If no charging location is available, we cannot send this agent anywhere.
  if(!nearest.first){
    std::cout << "Didn't find a charging location." << std::endl;
    return;
  }
  std::unique_ptr<Boundary> location(nearest.first->Clone());
  DispatchTo(_member, std::move(location));
}

bool
BatteryConstrainedGroup::
ClearToPlan(Agent* const _member){
  auto task = _member->GetTask();
  for(auto agentPair : m_helperMap){
    if(agentPair.second != _member)
      continue;
    return !task->EvaluateGoalConstraints({agentPair.first->GetRobot()->
                  GetDynamicsModel()->GetSimulatedState()});
  }
  return true;
}

/*----------------------------------Task Management----------------------------------*/

void
BatteryConstrainedGroup::
AssignTaskWorker(Agent* const _member) {
  // If this worker has no task, get a new one from the MPProblem
  if(!_member->GetTask()){
    auto tasks = m_robot->GetMPProblem()->GetTasks(m_robot);
    for(auto& task : tasks){
       if(task->IsStarted())
         continue;
       _member->SetTask(task);
       task->SetStarted();
       break;
    }
    if(tasks.size() < 1){
      std::cout << "No Tasks left in the problem" << std::endl;
    }
  }
}

// TODO: Create a function to pick the best/closest paused task to assign.
// TODO: Or, use GetNearestHelper and see if _member is nearest
//       If it is nearest, assign the task.
//       Otherwise, do nothing.
void
BatteryConstrainedGroup::
AssignTaskHelper(Agent* const _member) {
  if(m_proactive){
    AssignProactiveHelperTask(_member);
    return;
  }
  
  // If there are no paused tasks, we have nothing to assign to this helper.
  if(m_pausedTasks.empty())
    return;

  // Move the paused task to the list of switching tasks.
  PausedTask pausedTask = m_pausedTasks.front();
  m_pausedTasks.pop_front();

  PathFollowingChildAgent* childAgent = static_cast<PathFollowingChildAgent*>(_member);
  childAgent->SetPausedTask(pausedTask.m_task);
  // Set the members's new priortiy and role. 
  //SetPriority(_member, pausedTask.m_priority);
  //SetRole(_member, Worker);
  // Assign this member as the helper for the waiting agent.
  //TakeOverTask(_member, pausedTask);/
  Agent* previousOwner = pausedTask.m_previousOwner;
  m_helperMap[previousOwner] = _member;

  // TODO: Need to adjust the task's start constraint to be previous owner's
  std::shared_ptr<MPTask> newTask(new MPTask(m_robot));
  // Create bounding sphere around the old worker robot and select a valid
  // starting configuration within the boundary 
  auto workerPos = previousOwner->GetRobot()->
    GetDynamicsModel()->GetSimulatedState().GetPosition();
  auto combinedRadius = 1.5 * (_member->GetRobot()->GetMultiBody()->GetBoundingSphereRadius() +
      previousOwner->GetRobot()->GetMultiBody()->GetBoundingSphereRadius());
  CSpaceBoundingSphere boundingSphere(workerPos, combinedRadius);  
  std::vector<Cfg> goalPoints;

  /// @TODO Change this to a parameter instead of hardcoding sampler label.
  auto sampler = m_library->GetSampler("UniformRandomFreeRobot");
  size_t numNodes = 1, numAttempts = 100;
  sampler->Sample(numNodes, numAttempts, &boundingSphere, 
      std::back_inserter(goalPoints));

  if(goalPoints.empty()) 
    throw RunTimeException(WHERE, "No valid starting configuration for the PausedTask.");

  Cfg validCfg = goalPoints[0];
  auto goal = std::unique_ptr<CSpaceConstraint>
    (new CSpaceConstraint(m_robot, validCfg)); 

  //TODO: Need to insert goal constraint at the beginning
  newTask->AddGoalConstraint(std::move(goal));
  _member->SetTask(newTask);
  SetRole(_member, GoingToHelp);
  SetPriority(_member, GetPriority(_member) + 1000);
  // Clear the charging location that member was on.
  
  ClearChargingLocation(_member);
}


void 
BatteryConstrainedGroup::
AssignProactiveHelperTask(Agent* const _member){
  std::cout << "Number of battery breaks stored: " << m_batteryBreaks.size() << std::endl;
  if(m_batteryBreaks.empty())
    return;
  BatteryBreak bBreak = (*m_batteryBreaks.begin()).second;
  Agent* worker = (*m_batteryBreaks.begin()).first;
  m_helperMap[worker] = _member;

  std::shared_ptr<MPTask> task(new MPTask(m_robot));
  task->SetArrivalTime(bBreak.GetTime());

  double combinedRadius = 1.5 * (_member->GetRobot()->GetMultiBody()->GetBoundingSphereRadius() +
    worker->GetRobot()->GetMultiBody()->GetBoundingSphereRadius());

  CSpaceBoundingSphere boundingSphere(bBreak.GetPlace().GetPosition(), combinedRadius);  
  std::vector<Cfg> goalPoints;

  auto sampler = m_library->GetSampler("UniformRandomFreeRobot");
  size_t numNodes = 1, numAttempts = 100;
  sampler->Sample(numNodes, numAttempts, &boundingSphere, 
      std::back_inserter(goalPoints));
  //TODO changes to allow it to plan again instead. Maybe delete current save in
  // helper map or move helper map assignment down to end of function.
  if(goalPoints.empty()) 
    throw RunTimeException(WHERE, "No valid starting configuration for the PausedTask.");

  Cfg validCfg = goalPoints[0];
  auto goal = std::unique_ptr<CSpaceConstraint>
    (new CSpaceConstraint(m_robot, validCfg)); 
  
  task->AddGoalConstraint(std::move(goal));

  SetRole(_member, GoingToHelp);
  _member->SetTask(task);
  
}

void
BatteryConstrainedGroup::
AssignTaskWaiting(Agent* const _member){
  auto iter = m_helperMap.find(_member);
  // If the helper has arrived, send the waiting agent back to charge.
  if(iter != m_helperMap.end()) { 
    Agent* helper = iter->second;
    if(InHandoffProximity(_member, helper)) {
      SetRole(_member, ReturningToCharge);
      // TODO: This 0 priority could be a problem later, since we are assuming the
      //       robot has just enough battery to reach a charging station.
      SetPriority(_member, GetPriority(_member) + 500);
      //PerformHelperSwap(helper, _member);
      helper->SetTask(nullptr);
      //m_helperMap.erase(iter);
    }
  }
}



void
BatteryConstrainedGroup::
PerformHelperSwap(Agent* const _helper, Agent* _worker){
  PathFollowingChildAgent* childAgent = static_cast<PathFollowingChildAgent*>(_helper);
  std::cout << "Perform Helper Swap" << std::endl;
  if(m_proactive){
    for(auto it = m_pausedTasks.begin(); it != m_pausedTasks.end(); it++){
      PausedTask pausedTask = *it;
      if(pausedTask.m_previousOwner != _worker)
        continue;
      //SetPriority(_helper, pausedTask.m_priority);
      SetRole(_helper, Worker);
      //SetRole(_helper, Charging);//used to test return to charge function
                                 //want this guy to stop doing stuff 
      //_helper->SetTask(pausedTask.m_task);

      if(pausedTask.m_task){
        _helper->SetTask(pausedTask.m_task);
      }
      else {
        AssignTaskWorker(_helper);
      }
      m_pausedTasks.erase(it);
      break;
    }
  }
  else {
    std::cout << "Stored Task: " << childAgent->GetPausedTask() << std::endl;;  
    SetRole(_helper, Worker);
    //SetRole(_helper, Charging);
    if(childAgent->GetPausedTask()){
      _helper->SetTask(childAgent->GetPausedTask());
    }
    else {
      AssignTaskWorker(_helper);
    }
  }
}


void
BatteryConstrainedGroup::
BatteryCheck(Agent* const _member){
  if(GetRole(_member) == Worker){
    PathFollowingChildAgent* childAgent = static_cast<PathFollowingChildAgent*>(_member);
    if(childAgent->IsBatteryLow()){
 
      PausedTask task;
      task.m_task = _member->GetTask();

      if(task.m_task->EvaluateGoalConstraints({_member->GetRobot()->
                  GetDynamicsModel()->GetSimulatedState()})){
        task.m_task->SetCompleted();
        task.m_task = nullptr;
      }
      else {
        std::cout << _member->GetRobot()->GetLabel() << " is not on the goal" << std::endl;
      }

      task.m_previousOwner = _member;
      task.m_priority = GetPriority(_member);
      m_pausedTasks.push_back(task);
      // If we need to wait for a helper handoff, wait here.
      if(m_handoff){
        std::cout << _member->GetRobot()->GetLabel() << " WAITING FOR HELP" << std::endl;
        _member->SetTask(nullptr);
        SetRole(_member, WaitingForHelp);
        SetPriority(_member, GetPriority(_member) - 1000);
      }
      // If we can stop the task without waiting, go charge.
      else{
        SetRole(_member, ReturningToCharge);
        SetPriority(_member, GetPriority(_member) + 500);
        //GoToCharge(_member);
        _member->SetTask(nullptr);
      }
    }
  }
}


double
BatteryConstrainedGroup::
GetCurrentTime(){
  return m_currentTime;
}

