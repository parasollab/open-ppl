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
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
}


void
PlanningAgent::
Step(const double _dt) {
  Initialize();

  // If the agent is planning, skip this step.
  if(m_planning)
    return;

  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

  // If we have no task, select the next one.
  if(!GetTask() && !SelectTask()) {
    /// @TODO: Helper's battery level is being depleted here, must fix this.
    // If no incomplete tasks remain, we are done.
    if(m_debug)
      std::cout << "Completed all tasks, halting robot." << std::endl;
    PauseAgent(1);
    return;
  }

  // If we have no plan, generate a plan.
  if(!HasPlan()) {
    GeneratePlan();
    return;
  }

  // Evaluate task progress. If task is still valid, continue execution.
  if(EvaluateTask())
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


void
PlanningAgent::
SetTask(MPTask* const _task) {
  ClearPlan();
  Agent::SetTask(_task);
}

/*------------------------------- Planning -----------------------------------*/

bool
PlanningAgent::
IsPlanning() const {
  return m_planning;
}


size_t
PlanningAgent::
GetPlanVersion() const {
  return m_planVersion;
}

/*---------------------------- Planning Helpers ------------------------------*/

void
PlanningAgent::
GeneratePlan() {
  m_planning = true;
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));

  m_thread = std::thread([this, problemCopy](){
    this->WorkFunction(problemCopy);
  });

  ++m_planVersion;

  // Detach thread so that it automatically joins on completion.
  m_thread.detach();
}


void
PlanningAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // Initialize the solution.
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));

  // Create a plan with PMPL.
  m_library->Solve(_problem.get(), GetTask(), m_solution.get());

  // Retarget the library's problem back on the global copy so that later uses
  // of m_library which depend on the problem will not crash.
  m_library->SetMPProblem(m_robot->GetMPProblem());

  m_planning = false;
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
