#include "RoadmapFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"


/*------------------------------ Construction --------------------------------*/

RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r) : Agent(_r) { }


RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r, const RoadmapFollowingAgent& _a)
  : Agent(_r, _a) { }


RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r, XMLNode& _node) : Agent(_r) {
  // Currently there are no parameters. Parse XML options here.
}


std::unique_ptr<Agent>
RoadmapFollowingAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<RoadmapFollowingAgent>(
      new RoadmapFollowingAgent(_r, *this));
}


RoadmapFollowingAgent::
~RoadmapFollowingAgent() {
  RoadmapFollowingAgent::Uninitialize();
}

/*------------------------------ Agent Interface -----------------------------*/

void
RoadmapFollowingAgent::
Initialize() {
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = new MPLibrary(xmlFile);

  /// @TODO Choose the task intelligently rather than just taking the first one.
  auto task = problem->GetTasks(m_robot).front().get();
  this->SetTask(task);

  // Create a new solution object to hold a plan for this agent.
  m_solution = new MPSolution(m_robot);

  // Use the planning library to find a path.
  m_library->Solve(problem, task, m_solution);

  // If the path is empty, PMPL has failed somehow.
  if(m_solution->GetPath()->Size() == 0)
    throw RunTimeException(WHERE, "PMPL produced an empty path.");

  // Initialize the current subgoal.
  m_currentSubgoal = m_solution->GetPath()->VIDs().begin();
}


void
RoadmapFollowingAgent::
Step(const double _dt) {
  Initialize();

  // Do nothing if path traversal is complete.
  if(PathCompleted())
    return;

  // If there are no steps remaining for the current control(s), select next
  // edge.
  if(m_stepsRemaining == 0) {
    SetNextSubgoal();

    // If we have reached the end of the path, halt the robot and return.
    if(PathCompleted()) {
      // Warning: Halt() doesn't respect the dynamics of the simulation and is
      // only to be used for visual verification of the path in the simulator.
      this->Halt();
      return;
    }

    SetNextControls();
  }

  if(m_debug)
    CheckRobot();
  ApplyCurrentControls();
}


void
RoadmapFollowingAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  delete m_solution;
  delete m_library;

  m_solution = nullptr;
  m_library  = nullptr;
}

/*-------------------------------- Helpers -----------------------------------*/

void
RoadmapFollowingAgent::
CheckRobot() const {
  // Find the expected configuration for the robot.
  const Cfg* expected;
  const size_t edgeIndex = m_edge->GetTimeSteps() - m_stepsRemaining;
  if(edgeIndex == 0)
    // We are just starting this edge, so the robot is at the previous subgoal.
    expected = &m_solution->GetRoadmap()->GetGraph()->
        GetVertex(*(m_currentSubgoal - 1));
  else
    // The robot should be at an intermediate configuration.
    expected = &(m_edge->GetIntermediates()[edgeIndex - 1]);

  // Find the current configuration for the robot.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();

  // Compare.
  Cfg::PrintRobotCfgComparisonInfo(std::cout, *expected, current);

  /// @note Even though we are following a roadmap, there will be some
  ///       difference between the simulated robot and the roadmap after the
  ///       first rotation occurs. This is because of unavoidable round-off
  ///       error related to coordinate frame transformations and (separately)
  ///       euler-angle to rotation matrix conversions.
}


void
RoadmapFollowingAgent::
ApplyCurrentControls() {
  const auto& currentControls = m_edge->GetControlSet();
  // Ensure we have at least one control to follow.
  if(currentControls.empty())
    throw RunTimeException(WHERE, "Control set is empty - agent does not know "
        "what to do.");

  if(m_debug)
    std::cout << "Continuing same control(s) for " << m_stepsRemaining
              << " more steps."
              << std::endl;

  --m_stepsRemaining;

  // Apply each control.
  for(size_t i = 0; i < currentControls.size(); ++i) {
    auto& control = currentControls[i];
    control.Execute();

    if(m_debug)
      std::cout << "\tApplying control " << i << ": " << control
                << std::endl;
  }
}


void
RoadmapFollowingAgent::
SetNextSubgoal() {
  ++m_currentSubgoal;
}


void
RoadmapFollowingAgent::
SetNextControls() {
  // Get the next edge in the roadmap path.
  auto previousSubgoal = m_currentSubgoal - 1;

  typename GraphType::edge_descriptor ed(*previousSubgoal, *m_currentSubgoal);
  typename GraphType::vertex_iterator vi;
  typename GraphType::adj_edge_iterator ei;

  const bool found = m_solution->GetRoadmap()->GetGraph()->find_edge(ed, vi, ei);

  // Assert that we found the edge.
  if(!found)
    throw RunTimeException(WHERE, "Couldn't find next edge in roadmap from VID "
        + std::to_string(*previousSubgoal) + " to "
        + std::to_string(*m_currentSubgoal) + ".");

  // Store the edge and set the number of steps.
  m_edge = &ei->property();
  m_stepsRemaining = m_edge->GetTimeSteps();

  // If there is a hardware robot attached to our simulation, send it the
  // commands also.
  auto hardwareInterface = static_cast<QueuedHardwareInterface*>(m_robot->GetHardwareInterface("base"));
  if(hardwareInterface)
    hardwareInterface->EnqueueCommand(m_edge->GetControlSet(),
        m_stepsRemaining * m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes());

  if(m_debug) {
    std::cout << "New controls selected for the next " << m_stepsRemaining
              << " steps:";
    for(const auto& c : m_edge->GetControlSet())
      std::cout << "\n\t" << c;
    std::cout << std::endl;
  }
}


bool
RoadmapFollowingAgent::
PathCompleted() const noexcept {
  return m_currentSubgoal == m_solution->GetPath()->VIDs().end();
}

/*----------------------------------------------------------------------------*/
