#include "RoadmapFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"


/*------------------------------ Construction --------------------------------*/

RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r) : PlanningAgent(_r) { }


RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r, const RoadmapFollowingAgent& _a)
  : PlanningAgent(_r, _a) { }


RoadmapFollowingAgent::
RoadmapFollowingAgent(Robot* const _r, XMLNode& _node) : PlanningAgent(_r) {
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
Uninitialize() {
  if(!m_initialized)
    return;
  PlanningAgent::Uninitialize();

  ClearPlan();
}

/*--------------------------------- Planning ---------------------------------*/

bool
RoadmapFollowingAgent::
HasPlan() const {
  return m_solution->GetPath()->Size() != 0;
}


void
RoadmapFollowingAgent::
ClearPlan() {
  m_solution->GetPath()->Clear();
  m_currentSubgoal = VIDIterator();
  m_edge = nullptr;
}

/*----------------------------- Planning Helpers -----------------------------*/

void
RoadmapFollowingAgent::
GeneratePlan() {
  // Create a new plan.
  PlanningAgent::GeneratePlan();

  // If the path is empty, PMPL has failed somehow.
  if(m_solution->GetPath()->Size() == 0)
    throw RunTimeException(WHERE, "PMPL produced an empty path.");

  // Initialize the current subgoal.
  m_currentSubgoal = m_solution->GetPath()->VIDs().begin();
}

/*------------------------------ Task Helpers -------------------------------*/

bool
RoadmapFollowingAgent::
EvaluateTask() {
  // Increment the subgoal and check if the path is complete.
  ++m_currentSubgoal;

  const bool complete = m_currentSubgoal == m_solution->GetPath()->VIDs().end();
  if(complete) {
    if(m_debug)
      std::cout << "Reached the end of the path." << std::endl;
    GetTask()->SetCompleted();
    SetTask(nullptr);
    ClearPlan();
  }

  return !complete;
}


void
RoadmapFollowingAgent::
ExecuteTask(const double) {
  if(m_debug) {
    // Find the expected configuration for the robot.
    const Cfg* expected;
    const size_t edgeIndex = m_edge->GetTimeSteps() - m_stepsRemaining;

    // If we are just starting this edge, the robot is at the previous subgoal.
    if(edgeIndex == 0)
      expected = &m_solution->GetRoadmap()->GetGraph()->
          GetVertex(*(m_currentSubgoal - 1));
    // Otherwise, the robot should be at an intermediate configuration.
    else
      expected = &(m_edge->GetIntermediates()[edgeIndex - 1]);

    const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();

    Cfg::PrintRobotCfgComparisonInfo(std::cout, *expected, current);
  }

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

  // Store the edge and enqueue the corresponding command.
  m_edge = &ei->property();
  ExecuteControls(m_edge->GetControlSet(), m_edge->GetTimeSteps());
}

/*----------------------------------------------------------------------------*/
