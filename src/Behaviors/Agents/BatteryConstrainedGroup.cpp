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

  // Set up the group members.
  int priority = 1;
  for(const auto& memberLabel : m_memberLabels) {
    Robot* member = problem->GetRobot(memberLabel);

    // We are assuming that all member robots have a compatible agent type.
    // Throw an exception if not.
    /// @TODO Generalize code so that this is not necessary.
    Agent* memberAgent = member->GetAgent();
    {
      PathFollowingChildAgent* a = dynamic_cast<PathFollowingChildAgent*>(
          memberAgent);
      if(!a)
        throw RunTimeException(WHERE, "Incompatible agent type specified for "
            "group member '" + memberLabel + "'.");
      m_memberAgents.push_back(a);

      a->m_parentAgent = this;
    }

    // Set the initial role for this member.
    SetRole(memberAgent, m_initialRoles[memberLabel]);

    // Set the initial priority.
    SetPriority(memberAgent, isWorker ? 1000 + priority++ : 0);
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
  auto task = sharedRoadmapTask.get();
  problem->AddTask(std::move(sharedRoadmapTask));

  m_library->Solve(problem, task, m_solution);
  task->SetCompleted();
}


void
BatteryConstrainedGroup::
Step(const double _dt) {
  Initialize();

  /// @TODO Verify we are still going to do things this way.
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

/*-------------------------- Coordinator Interface ---------------------------*/

void
BatteryConstrainedGroup::
AssignTask(Agent* const _member) {
  /// @TODO
}


void
BatteryConstrainedGroup::
ArbitrateCollision(const vector<Robot*>& _robots) {
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
  // Get _member's current location.
  auto robot = _member->GetRobot();
  const Cfg memberPos = robot->GetDynamicsModel()->GetSimulatedState();

  // Compare with all helpers and return the nearest.
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


void
BatteryConstrainedGroup::
DispatchTo(Agent* const _member, std::unique_ptr<Boundary>&& _where) {
  // Create a task to send the member to the desired location. Use the
  // coordinator robot because this is shared-roadmap planning.
  std::unique_ptr<MPTask> task(new MPTask(m_robot));
  std::unique_ptr<BoundaryConstraint> destination(
      new BoundaryConstraint(m_robot, std::move(_where))
  );
  task->AddGoalConstraint(std::move(destination));

  // Set the member's current task.
  _member->SetTask(task.get());

  // Add the task to the MPProblem.
  m_robot->GetMPProblem()->AddTask(std::move(task));
}


bool
BatteryConstrainedGroup::
InHandoffProximity(Agent* const _member1, Agent* const _member2) {
  auto robot1 = _member1->GetRobot(),
       robot2 = _member2->GetRobot();

  auto cfg1 = robot1->GetDynamicsModel()->GetSimulatedState(),
       cfg2 = robot2->GetDynamicsModel()->GetSimulatedState();

  auto dm = m_library->GetDistanceMetric("euclidean");

  const double distance = dm->Distance(cfg1, cfg2);

  // Use twice the average diameter as the handoff distance.
  const double threshold = 2. *
    (robot1->GetMultiBody()->GetBoundingSphereRadius() +
     robot2->GetMultiBody()->GetBoundingSphereRadius());

  return distance < threshold;
}

/*---------------------------- Charging Locations ----------------------------*/

void
BatteryConstrainedGroup::
InitializeChargingLocations() {
  for(Agent* const agent : m_memberAgents) {
    // Skip agents which are neither helpers nor charging.
    if(!GetRole(agent) == Helper and !GetRole(agent) == Charging)
      continue;

    // Get the position of the member.
    auto position = agent->GetRobot()->GetDynamicsModel()->GetSimulatedState();

    // Create a new charging boundary using this position.
    std::unique_ptr<CSpaceBoundingBox> boundary(2);
    boundary->ShrinkToPoint(position);
    m_chargingLocations.emplace_back(std::move(boundary), agent);
  }
}


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
