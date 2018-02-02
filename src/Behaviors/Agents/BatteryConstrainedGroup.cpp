#include "BatteryConstrainedGroup.h"

#include <limits>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "Behaviors/Agents/PathFollowingChildAgent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"

/*------------------------------ Construction --------------------------------*/

BatteryConstrainedGroup::
BatteryConstrainedGroup(Robot* const _r) : Agent(_r) {
}


BatteryConstrainedGroup::
BatteryConstrainedGroup(Robot* const _r, XMLNode& _node) : Agent(_r, _node) {
  // Parse the labels of the group members.
  for(auto& child : _node) {
    // Parse the robot label.
    const std::string memberLabel = _node.Read("label", true, "",
        "The label of the member robot.");
    m_memberLabels.push_back(memberLabel);

    // Parse the initial role.
    std::string role = _node.Read("role", true, "", "This robot's initial "
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
      throw ParseException(_node.Where(), "Unrecognized role '" + role + "'.");

    m_initialRoles[memberLabel] = memberRole;
  }

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

  // Set up the group member structures.
  int priority = 1;
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);

    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    /// @TODO Generalize code so that this is not necessary.
    Agent* memberAgent = member->GetAgent();
    PathFollowingChildAgent* a = dynamic_cast<PathFollowingChildAgent*>(memberAgent);
    if(!a)
      throw RunTimeException(WHERE, "Incompatible agent type specified for "
          "group member '" + memberLabel + "'.");
    m_memberAgents.push_back(a);

    a->m_parentRobot = m_robot;
    a->m_parentAgent = this;

    // Set the initial role for this member.
    SetRole(a, m_initialRoles[memberLabel]);

    // Set the initial priority.
    SetPriority(a, isWorker ? 1000 + priority++ : 0);

    /// @TODO Currently we assume that the starting location for each helper is a
    ///       charging location. We need to implement a set of labeled regions
    ///       in a problem to define charging areas independently.
    /// @TODO We currently assume that charging locations are XY regions which
    ///       extend infinitely in the z-dimension.
    if(GetRole(a) == Helper or GetRole(a) == Charging) {
      // Get the position of the member.
      auto position = robot->GetDynamicsModel()->GetSimulatedState();

      std::unique_ptr<CSpaceBoundingBox> boundary(2);
      boundary->ShrinkToPoint(position);

      // Initialize this charging location with the helper/charging member.
      m_chargingLocations.emplace_back(std::move(boundary), a);
    }
  }

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  // Generate the shared roadmap.
  std::unique_ptr<MPTask> sharedRoadmap(new MPTask(m_robot));
  m_library->Solve(problem, sharedRoadmap.get(), m_solution);
  sharedRoadmap->SetCompleted();
}


void
BatteryConstrainedGroup::
Step(const double _dt) {
  Initialize();

  // Nothing else for coordinator to do - children will be stepped by main
  // simulation loop.
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

/*--------------------------- Coordinator Functions --------------------------*/

void
BatteryConstrainedGroup::
ArbitrateCollision(const vector<Robot*>& _robots){
  /// @TODO  
  //if(m_robot->GetAgent()->m_priority < robot->GetAgent()->m_priority){
  //  m_shouldHalt = true;
  //  return true;
  //}
  //else{
  //  m_shouldHalt = false;
  //  coll = true;
  //}
}

/*---------------------------- Coordinator Helpers ---------------------------*/

std::vector<Cfg>
BatteryConstrainedGroup::
MakeNextPlan(MPTask* const _task, const bool _collisionAvoidance) {
  nonstd::timer clock;

  if(_collisionAvoidance) {
    clock.restart();
    m_library->Solve(m_robot->GetMPProblem(), _task, m_solution,
        "LazyPRM", LRand(), "LazyCollisionAvoidance");
    clock.stop();
    m_lazyTime += (clock.elapsed()/1e9);
  }
  else {
    clock.restart();
    m_library->Solve(m_robot->GetMPProblem(), _task, m_solution);
    clock.stop();
    m_prmTime += (clock.elapsed()/1e9);
  }

  std::ofstream ofs("output.txt");
  ofs << "Lazy Time: " << m_lazyTime << "\nBasic Time: " << m_prmTime << endl;
  return m_solution->GetPath()->Cfgs();
}


void
BatteryConstrainedGroup::
SetNextChildTask() {
  for(auto agent : m_memberAgents) {
    auto agentRobot = agent->GetRobot();

    // Get a random roadmap point as the new goal for this agent, and also get
    // the current position.
    const auto& newGoal = GetRandomRoadmapPoint(agentRobot->GetLabel());
    using CfgType = decltype(newGoal);
    const CfgType currentPos = agentRobot->GetDynamicsModel()->GetSimulatedState();

    // Create the task from the parent robot so that we correctly use the shared
    // roadmap (build on parent robot).
    auto start = new CSpaceConstraint(m_robot, currentPos);
    auto goal = new CSpaceConstraint(m_robot, newGoal);

    auto task = new MPTask(m_robot);
    task->AddStartConstraint(start);
    task->AddGoalConstraint(goal);

    agent->SetCurrentTask(task);
  }
}

/*--------------------------- Member Management ------------------------------*/

void
BatteryConstrainedGroup::
SetPriority(Agent* const _a, const size_t _priority) {
  m_memberPriorities[_a] = _priority;
}


size_t
BatteryConstrainedGroup::
GetPriority(Agent* const _a) {
  return m_memberPriorities[_a];
}


void
BatteryConstrainedGroup::
SetRole(Agent* const _a, const Role _r) {
  m_roleMap[_a] = _r;
}


BatteryConstrainedGroup::Role
BatteryConstrainedGroup::
GetRole(Agent* const _a) const {
  return m_roleMap[_a];
}


std::vector<Robot*>
BatteryConstrainedGroup::
GetHelpers() {
  std::vector<Robot*> output;

  for(auto member : m_memberAgents)
    if(GetRole(member) == Helper)
      output.push_back(member);

  return output;
}


Agent*
BatteryConstrainedGroup::
GetNearestHelper(Agent* const _member) {
  auto robot = _member->GetRobot();
  const Cfg memberPos = robot->GetDynamicsModel()->GetSimulatedState();

  double nearestDistance = std::numeric_limits<double>::max();
  Agent* nearestHelper = nullptr;

  auto dm = m_library->GetDistanceMetric("connectedFreeSpace");

  for(auto helper : GetHelpers()) {
    const Cfg helperPos = helper->GetDynamicsModel()->GetSimulatedState();
    const double distance = dm->Distance(memberPos, helperPos);

    if(distance < nearestDistance) {
      nearestDistance = distance;
      nearestHelper = helper->GetAgent();
    }
  }

  return nearestHelper;
}

/*---------------------------- Charging Locations ----------------------------*/

std::pair<Boundary*, double>
BatteryConstrainedGroup::
FindNearestChargingLocation(Agent* const _a) {
  auto robot = _a->GetRobot();
  auto currentPos = robot->GetDynamicsModel()->GetSimulatedState();

  Boundary* bestChargingLocation = nullptr;
  double bestDistance = std::numeric_limits<double>::max();

  /// @TODO Create this distance metric during initialization if it does not
  /// already exist.
  auto dm = m_library->GetDistanceMetric("connectedFreeSpace");

  // Find the closest charging location to m_robot
  for(const auto& chargingLocation : m_chargingLocations) {
    // Skip occupied locations.
    if(chargingLocation.second)
      continue;

    // Get a random configuration within the charging region.
    Boundary* region = chargingLocation.first.get();
    Cfg chargingCfg(robot);
    chargingCfg.SetData(region->GetRandomPoint());

    // Get the freespace distance.
    const double distance = dm->Distance(currentPos, chargingCfg);

    if(distance < bestDistance) {
      bestDistance = distance;
      bestChargingLocation = region;
    }
  }

  return {bestChargingLocation, bestDistance};
}


bool
BatteryConstrainedGroup::
IsAtChargingStation(Agent* const _member) {
  auto nearest = FindNearestChargingLocation(_member);
  Boundary* location = nearest.first;
  const double distance = nearest.second;

  if(!location)
    return false;

  // Define a distance threshold. We say a member is 'at' a charging location if
  // it is within this ammount.
  static constexpr double threshold = .15;
  
  return distance <= threshold;
}

/*----------------------------------------------------------------------------*/
