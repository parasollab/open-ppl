#include "PlanningAgent.h"

#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MetricUtils.h"

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

  // Initialize a clock to track this agent's total planning time. This is done
  // to ensure that the stat's clock map isn't adding elements across threads.
  const std::string clockName = "Planning::" + m_robot->GetLabel();
  Simulation::GetStatClass()->ClearClock(clockName);
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
  if(!HasPlan()) {
    PauseAgent(1);
    GeneratePlan();
    return;
  }

  // Evaluate task progress. If task is still valid, continue execution.
  if(EvaluateTask())
    ExecuteTask(_dt);
  else
    PauseAgent(1);
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

  if(m_roadmapVisualID)
    Simulation::Get()->RemoveRoadmap(m_roadmapVisualID);

  m_roadmapVisualID = 0;
}


void
PlanningAgent::
SetTask(std::shared_ptr<MPTask> const _task) {
  ClearPlan();
  Agent::SetTask(_task);
}

/*------------------------------- Planning -----------------------------------*/

void
PlanningAgent::
ClearPlan() {
  if(HasPlan())
    ++m_planVersion;
  if(!GetTask())
    return;

}


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

void 
PlanningAgent::
SetMPLibrary(MPLibrary* _library){
  m_library = std::unique_ptr<MPLibrary>(_library);
}

/*---------------------------- Planning Helpers ------------------------------*/

void
PlanningAgent::
GeneratePlan() {
  m_planning = true;

  // Sets tasks' start constraint to the robot's current position
  auto position = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, position));
  
  GetTask()->SetStartConstraint(std::move(start));
  // Create a copy of the problem so that we can use the data objects in planning
  // without affecting the rest of the simulation.
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));

  // Start running the work function in another thread.
  m_thread = std::thread([this, problemCopy]() {
    // Track planning time by robot label.
    MethodTimer mt(Simulation::GetStatClass(),
        "Planning::" + this->m_robot->GetLabel());

    this->WorkFunction(problemCopy);

    // Retarget the library's problem back on the global copy so that later uses
    // of m_library which depend on the problem will not crash.
    this->m_library->SetMPProblem(this->m_robot->GetMPProblem());

    // Update plan version and flag.
    ++m_planVersion;
    m_planning = false;
  });

  // Detach thread so that it automatically joins on completion.
  m_thread.detach();
}


void
PlanningAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // Initialize the solution.
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
  
  // add DrawableRoadmap to be drawn
  m_roadmapVisualID = Simulation::Get()->AddRoadmap(m_solution->GetRoadmap()->GetGraph(),
      glutils::color::green);

  // Create a plan with PMPL.
  m_library->Solve(_problem.get(), GetTask().get(), m_solution.get());
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
