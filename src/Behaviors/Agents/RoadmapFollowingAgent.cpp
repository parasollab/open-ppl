#include "RoadmapFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletModel.h"

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
  Uninitialize();
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
  PlanningAgent::ClearPlan();
  m_solution->GetPath()->Clear();
  m_currentSubgoal = VIDIterator();
  m_edge = nullptr;
}

/*----------------------------- Planning Helpers -----------------------------*/

void
RoadmapFollowingAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // Create a new plan.
  PlanningAgent::WorkFunction(_problem);

  // If the path is empty, PMPL has failed somehow.
  if(m_solution->GetPath()->Size() == 0)
    throw RunTimeException(WHERE) << "PMPL produced an empty path.";

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
    GetTask()->GetStatus().complete();
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
      expected = &m_solution->GetRoadmap()->GetVertex(*(m_currentSubgoal - 1));
    // Otherwise, the robot should be at an intermediate configuration.
    else
      expected = &(m_edge->GetIntermediates()[edgeIndex - 1]);

    const Cfg current = m_robot->GetSimulationModel()->GetState();

    std::cout << "Check robot position:"
              << "\n\tCurrent:    " << current.PrettyPrint()
              << "\n\tExpected:   " << expected->PrettyPrint()
              << "\n\tDifference: " << (*expected - current).PrettyPrint()
              << "\n\t|Diff|:     " << (*expected - current).Magnitude()
              << std::endl;
  }

  // Get the next edge in the roadmap path.
  auto previousSubgoal = m_currentSubgoal - 1;

  typename RoadmapType::edge_descriptor ed(*previousSubgoal, *m_currentSubgoal);
  typename RoadmapType::vertex_iterator vi;
  typename RoadmapType::adj_edge_iterator ei;

  const bool found = m_solution->GetRoadmap()->find_edge(ed, vi, ei);

  // Assert that we found the edge.
  if(!found)
    throw RunTimeException(WHERE) << "Couldn't find next edge in roadmap from "
                                  << "VID " << *previousSubgoal
                                  << " to " << *m_currentSubgoal << ".";

  // Store the edge and enqueue the corresponding command.
  m_edge = &ei->property();
  ExecuteControls(m_edge->GetControlSet(), m_edge->GetTimeSteps());
}

/*----------------------------------------------------------------------------*/
