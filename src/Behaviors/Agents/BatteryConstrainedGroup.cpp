#include "BatteryConstrainedGroup.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "Behaviors/Agents/RoadmapFollowingAgent.h"
#include "MPProblem/Robot/DynamicsModel.h"


/*------------------------------ Construction --------------------------------*/

BatteryConstrainedGroup::
BatteryConstrainedGroup(Robot* const _r) : Agent(_r) {
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

  //TODO: Get distance thresholds from XML

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  // Generate the shared roadmap.
  auto task = problem->GetTasks(m_robot).front();
  m_library->Solve(problem, task, m_solution);

  auto childRobots = GetChildRobots();
  int priority = 1;

  // Initialize the agent for each child robot.
  for(auto robot : childRobots) {
    // Create an agent for this child.
    auto agent = new PathFollowingChildAgent(robot);
    robot->SetAgent(agent);

    // Set up priorities and charging locations based on the robot labels.
    if(IsHelper(robot)) {
      // Currently we assume that the starting location for each helper is a
      // charging location.
      auto helperPos = robot->GetDynamicsModel()->GetSimulatedState();
      m_chargingLocations.push_back(make_pair(helperPos, true));
      m_availableHelpers.push_back(robot);
      agent->m_priority = 0;
    }
    else {
      agent->m_priority = 1000 + priority++;
    }

    // Set up the shared roadmap and parent/child relationship.
    //agent->InitializeMPSolution(m_solution);
    agent->m_parentRobot = m_robot;
    agent->m_parentAgent = this;
    m_childAgents.push_back(agent);
  }

  // Initialize the set of unvisited configurations.
  InitializeUnvisitedCfgs();
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
  m_availableHelpers.clear();
  m_solution = nullptr;
  m_library  = nullptr;
}

/*--------------------------- Coordinator Functions --------------------------*/

std::vector<Robot*>&
BatteryConstrainedGroup::
GetHelpers() {
  return m_availableHelpers;
}


std::vector<pair<Cfg, bool>>&
BatteryConstrainedGroup::
GetChargingLocations() {
  return m_chargingLocations;
}


bool
BatteryConstrainedGroup::
IsHelper(Robot* const _r) const {
  return _r->GetLabel().compare(0, 6, "helper") == 0;
}


const Cfg
BatteryConstrainedGroup::
GetRandomRoadmapPoint() {
  Cfg empty(nullptr);
  if(m_unvisitedCfgs.empty()) {
    return empty;
  }
  else {
    Cfg point = m_unvisitedCfgs.back();
    m_unvisitedCfgs.pop_back();
    return point;
  }
}

/*---------------------------- Coordinator Helpers ---------------------------*/

std::vector<Cfg>
BatteryConstrainedGroup::
MakeNextPlan(MPTask* const _task, const bool _collisionAvoidance) {
  if(_collisionAvoidance) 
    m_library->Solve(m_robot->GetMPProblem(), _task, m_solution,
        "LazyPRM", LRand(), "LazyCollisionAvoidance");
  else 
    m_library->Solve(m_robot->GetMPProblem(), _task, m_solution);
    
  return m_solution->GetPath()->Cfgs();
}


void
BatteryConstrainedGroup::
SetNextChildTask() {
  for(auto agent : m_childAgents) {
    auto agentRobot = agent->GetRobot();

    // Get a random roadmap point as the new goal for this agent, and also get
    // the current position.
    const auto& newGoal = GetRandomRoadmapPoint();
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


std::vector<Robot*>
BatteryConstrainedGroup::
GetChildRobots() const {
  auto problem = m_robot->GetMPProblem();

  std::vector<Robot*> childRobots;
  for(auto i : problem->GetRobots()) {
    if(i->GetLabel() != m_robot->GetLabel())
      childRobots.push_back(i);
  }
  return childRobots;
}


void
BatteryConstrainedGroup::
InitializeUnvisitedCfgs() {
  /*for(typename GraphType::VI i = m_solution->GetRoadmap()->GetGraph()->begin();
      i!=m_solution->GetRoadmap()->GetGraph()->end(); i++) {
    Cfg cfg =  m_solution->GetRoadmap()->GetGraph()->GetVertex(i);
    if (!(std::find(m_unvisitedCfgs.begin(), m_unvisitedCfgs.end(),cfg)!=m_unvisitedCfgs.end()))
      m_unvisitedCfgs.push_back(cfg);
  }
  std::random_shuffle ( m_unvisitedCfgs.begin(), m_unvisitedCfgs.end() );*/

  Cfg point(m_robot);
  std::istringstream pointStream0("1 0 0 0 0 0");
  point.Read(pointStream0);
  m_unvisitedCfgs.push_back(point);
  std::istringstream pointStream("3 0 0 0 0 0");
  point.Read(pointStream);
  m_unvisitedCfgs.push_back(point);
  std::istringstream pointStream1("5 -2 0 0 0 0");
  point.Read(pointStream1);
  m_unvisitedCfgs.push_back(point);
  /*std::istringstream pointStream2("4.75 0 0 0 0 0");
  point.Read(pointStream2);
  m_unvisitedCfgs.push_back(point);
  std::istringstream pointStream3("4.87 -1.524 0 0 0 0");
  point.Read(pointStream3);*/
  //m_unvisitedCfgs.push_back(point);
}

/*----------------------------------------------------------------------------*/
