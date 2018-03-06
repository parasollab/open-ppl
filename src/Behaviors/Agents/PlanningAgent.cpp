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

  // If the agent is planning, skip this step.
  if(m_planning)
    return;

  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

  // If we have no task, select the next one.
  if(!GetTask() && !SelectTask()) {
    // If no incomplete tasks remain, we are done.
    if(m_debug)
      std::cout << "Completed all tasks, halting robot." << std::endl;
    PauseAgent(1);
    return;
  }

  // If we have no plan, generate a plan.
  if(!HasPlan()){
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


/*------------------------------- Planning -----------------------------------*/

bool
PlanningAgent::
IsPlanning() const {
  return m_planning;
}


/*---------------------------- Planning Versions -----------------------------*/

void
PlanningAgent::
SetPlanVersion(size_t _version) {
  m_planVersion = _version;
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
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));

  m_thread = std::thread([this, problemCopy](){
    this->WorkFunction(problemCopy);
  });

  // Detach thread so that it automatically joins on completion.
  m_thread.detach();
}

void 
PlanningAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // TODO: Stop trying to solve if it takes longer than t_max.
  m_library->Solve(_problem.get(), GetTask(), m_solution.get());
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
