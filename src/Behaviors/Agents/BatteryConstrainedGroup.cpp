#include "BatteryConstrainedGroup.h"

#include <limits>

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

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
  //double depletion = 0.20;

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
      /*auto hardwareInterface = static_cast<QueuedHardwareInterface*>
            (agent->GetRobot()->GetHardwareInterface("base")); //TODO Magic string
      if(hardwareInterface){
        agent->m_depletionRate = depletion;
      }
      else {
        agent->m_depletionRate = depletion;
      }
      depletion += 0.05;*/
    }

    // Set up the shared roadmap and parent/child relationship.
    //agent->InitializeMPSolution(m_solution);
    agent->m_parentRobot = m_robot;
    agent->m_parentAgent = this;
    m_childAgents.push_back(agent);
  }
      //auto helperPos = robot->GetDynamicsModel()->GetSimulatedState();

  CfgType chargingPoint(m_robot);
  auto newtask = m_robot->GetMPProblem()->GetTasks(m_robot).front();
  auto boundary = newtask->GetStartBoundary();
  chargingPoint.SetData(boundary->GetCenter());
  m_chargingLocations.push_back(make_pair(chargingPoint, false));

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


void
BatteryConstrainedGroup::
SetHelper(Robot* _r) {
  std::string label = _r->GetLabel();
  label.replace(0,6,"helper");
  _r->SetLabel(label);
}


void
BatteryConstrainedGroup::
SetWorker(Robot* _r) {
  std::string label = _r->GetLabel();
  label.replace(0,6,"worker");
  _r->SetLabel(label);
}


bool 
BatteryConstrainedGroup::
AllGoalsCompleted() {
  for(auto val: m_unvisitedCfgs)
    if(!val.second.empty())
      return false;
  return true;
}

void
BatteryConstrainedGroup::
AddGoal(Cfg& _cfg, const std::string& _robotLabel) {
  m_unvisitedCfgs[_robotLabel].push_back(_cfg);
}


const Cfg
BatteryConstrainedGroup::
GetRandomRoadmapPoint(std::string _label) {
  Cfg empty(nullptr);
  if(m_unvisitedCfgs[_label].empty()) {
    return empty;
  }
  else {
    Cfg point = m_unvisitedCfgs[_label].back();
    m_unvisitedCfgs[_label].pop_back();
    return point;
  }
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
  //Write basicTime and lazyTime to output file
  return m_solution->GetPath()->Cfgs();
}


void
BatteryConstrainedGroup::
SetNextChildTask() {
  for(auto agent : m_childAgents) {
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

  for(auto robot : GetChildRobots()) {
    auto tasks = m_robot->GetMPProblem()->GetTasks(robot);
    std::vector<Cfg> workerCfgs;
    for(auto task : tasks) {
      auto boundary = task->GetGoalBoundary();
      Cfg newCfg(m_robot);
      newCfg.SetData(boundary->GetCenter());
      workerCfgs.push_back(newCfg);
    }
    m_unvisitedCfgs[robot->GetLabel()] = workerCfgs;
  }
}

/*----------------------------------------------------------------------------*/
