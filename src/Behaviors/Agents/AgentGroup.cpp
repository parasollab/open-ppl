#include "AgentGroup.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "Behaviors/Agents/RoadmapFollowingAgent.h"
#include "MPProblem/Robot/DynamicsModel.h"


/*------------------------------ Construction --------------------------------*/

AgentGroup::
AgentGroup(Robot* const _r) : Agent(_r) {
}

AgentGroup::
~AgentGroup() {
  AgentGroup::Uninitialize();
}

/*------------------------------ Agent Interface -----------------------------*/

std::vector<Robot*>
AgentGroup::
GetChildRobots() {
  auto problem = m_robot->GetMPProblem();

  std::vector<Robot*> childRobots;
  for(auto i : problem->GetRobots()) {
    if(i->GetLabel() != m_robot->GetLabel())
      childRobots.push_back(i);
  }
  return childRobots;
}

std::vector<Robot*>&
AgentGroup::
GetHelpers() {
  return m_availableHelpers;
}


std::vector<pair<Cfg, bool>>&
AgentGroup::
GetChargingLocations() {
  return m_chargingLocations;
}

void
AgentGroup::
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

  /// @TODO Choose the task rather than just taking the first one.
  auto task = problem->GetTasks().front();

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(task->GetRobot());
  m_library->Solve(problem, task, m_solution);

  auto childRobots = GetChildRobots();
  int priority = 1;
  for (auto robot : childRobots){
    auto agent = new PathFollowingChildAgent(robot);
    if(robot->GetLabel().compare(0, 6, "helper") == 0) {
      cout << "Agent group is adding helpers " << endl;
      auto helperPos = robot->GetDynamicsModel()->GetSimulatedState();
      m_chargingLocations.push_back(make_pair(helperPos, true));
      m_availableHelpers.push_back(robot);
      agent->m_priority = 0;
    }
    else{
      agent->m_priority = 1000 + priority++;
    }
    agent->InitializeMpSolution(m_solution);
    agent->m_parentRobot = m_robot;
    agent->m_parentAgent = this;
    m_RobotGroup.push_back(agent);
    robot->SetAgent(agent);
  }

}


const Cfg&
AgentGroup::
GetRandomRoadmapPoint() const {
  const size_t randN = rand() % m_solution->GetRoadmap()->GetGraph()->get_num_vertices();

  typename GraphType::VI startPt = m_solution->GetRoadmap()->GetGraph()->begin();
  std::advance(startPt, randN);
  return m_solution->GetRoadmap()->GetGraph()->GetVertex(startPt);
}


void
AgentGroup::
Step(const double _dt) {
  Initialize();
  for(auto agent : m_RobotGroup)
    agent->Step(_dt);
}


void
AgentGroup::
SetNextChildTask() {
  for(auto agent : m_RobotGroup) {
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


void
AgentGroup::
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

/*----------------------------------------------------------------------------*/
