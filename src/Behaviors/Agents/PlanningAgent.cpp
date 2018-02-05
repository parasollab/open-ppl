#include "PlanningAgent.h"

#include <iostream>


/*------------------------------ Construction --------------------------------*/

PlanningAgent::
PlanningAgent(Robot* const _r) : Agent(_r) { }


PlanningAgent::
PlanningAgent(Robot* const _r, const PlanningAgent& _a) : Agent(_r, _a)
{ }


PlanningAgent::
PlanningAgent(Robot* const _r, XMLNode& _node) : Agent(_r) {
  // Currently there are no parameters. Parse XML options here.
}


PlanningAgent::
~PlanningAgent() = default;

/*----------------------------- Agent Interface ------------------------------*/

void
PlanningAgent::
Initialize() {
  // Guard against re-init.
  if(m_initialized)
    return;
  m_initialized = true;

  // Get problem info.
  auto problem = m_robot->GetMPProblem();
  const std::string& xmlFile = problem->GetXMLFilename();

  // Initialize the agent's planning library.
  m_library = std::unique_ptr<MPLibrary>(new MPLibrary(xmlFile));
}


void
PlanningAgent::
Step(const double _dt) {
  Initialize();

  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

  // If we have no task, select the next one and generate a plan.
  if(!GetTask()) {
    // If no incomplete tasks remain, we are done.
    if(!SelectTask()) {
      if(m_debug)
        std::cout << "Completed all tasks, halting robot." << std::endl;
      Halt();
      return;
    }
    GeneratePlan();
  }

  // Evaluate task progress. If not done, continue execution.
  if(!EvaluateTask())
    ExecuteTask(_dt);
}


void
PlanningAgent::
Uninitialize() {
  if(!m_initialized)
    return;
  m_initialized = false;

  m_library.reset();
  m_solution.reset();
  SetTask(nullptr);
}

/*---------------------------- Planning Helpers ------------------------------*/

void
PlanningAgent::
GeneratePlan() {
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));

  // Use the planning library to find a path. This works on a copy of the
  // problem to avoid concurrency issues.
  MPProblem problemCopy = *m_robot->GetMPProblem();
  m_library->Solve(&problemCopy, GetTask(), m_solution.get());
}

/*------------------------------ Task Helpers --------------------------------*/

bool
PlanningAgent::
SelectTask() {
  auto tasks = m_robot->GetMPProblem()->GetTasks(m_robot);

  // Return false if there are no unfinished tasks.
  if(tasks.empty()) {
    SetTask(nullptr);
    return false;
  }

  // Otherwise, choose the next one.
  SetTask(tasks.front());
  GetTask()->SetStarted();
  return true;
}

/*----------------------------------------------------------------------------*/
