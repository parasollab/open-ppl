#include "PlanningAgent.h"

#include "MPProblem/Robot/HardwareInterfaces/RobotCommandQueue.h"
#include "MPProblem/Robot/HardwareInterfaces/SensorInterface.h"
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

  // If the agent is planning or localizing, skip this step.
  if(IsPlanning() or IsLocalizing())
    return;

  // If the agent localized before this step, update the simulated state and
  // replan if necessary.
  if(!IsLocalizing() and m_localizeCount == 0) {
    auto hardware = m_robot->GetHardwareQueue();
    if(hardware) {
      auto sensor = hardware->GetSensor();
      if(sensor)
        UpdateSimulatedState(sensor->GetLastTransformations());
    }
  }

  // If the simulation has passed a set number of timesteps, localize.
  ++m_localizeCount;
  if(m_localizeCount > m_localizePeriod) {
    if(m_debug)
      std::cout << "Enqueueing localize command." << std::endl;
    Localize();
    m_localizeCount = 0;
  }

  // Wait for the previous controls to finish if they still have time remaining.
  if(ContinueLastControls())
    return;

  // If we have no task, select the next one.
  if(!GetTask() and !SelectTask()) {
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
  if(EvaluateTask()){
    ExecuteTask(_dt);
  }
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

/*------------------------------ Localization Helpers --------------------------------*/

void 
PlanningAgent::
UpdateSimulatedState(const std::vector<mathtool::Transformation>& _transformations) {

  // Do nothing if the robot saw no markers.
  if(_transformations.size() == 0) {
    if(m_debug)
      std::cout << "No Markers Found" << std::endl;
    return;
  }

  // Average the estimated state and update the robot's simulated position.
  double averageX = 0, averageY = 0;
  for(auto& t : _transformations) {
    averageX += t.translation()[0];
    averageY += t.translation()[1];
  }
  averageX /= _transformations.size();
  averageY /= _transformations.size();
  double averageT = ComputeRotation(_transformations);

  auto previousPos = m_robot->GetDynamicsModel()->GetSimulatedState();
  Cfg updatedPos(m_robot);
  updatedPos.SetLinearPosition(Vector3d(averageX, averageY, 0));
  updatedPos.SetAngularPosition(Vector3d(0, 0, averageT));

  // Ensure that the updated position is valid 
  if(!m_library->GetValidityChecker("pqp_solid")->IsValid(updatedPos, "cfg")) {
    if(m_debug)
      std::cout << "Ignoring Invalid Localization at: " << updatedPos << std::endl;
    return;
  }
    
  m_robot->GetDynamicsModel()->SetSimulatedState(updatedPos);
  
  auto dm = m_library->GetDistanceMetric("euclidean");
  const double distance = dm->Distance(previousPos, updatedPos);
  // If the error is too large, tell the agent to replan.
  if(distance > m_localizeErrorThreshold)
    ClearPlan();
    
  if(m_debug) {
    std::cout << "***\nSaw " << _transformations.size() << " markers:";
    for(auto& t : _transformations) {
      std::cout << "\n\t" << t.translation()
                << "\n\t" << t.rotation()
                << std::endl;
    }
    std::cout << "\nOld Simulated State: "
              << previousPos
              << "\nNew Simulated State: "
              << m_robot->GetDynamicsModel()->GetSimulatedState()
              << "\nDistance: "
              << distance
              << std::endl;
  }
}

double
PlanningAgent::
ComputeRotation(const std::vector<mathtool::Transformation>& _transformations) {
  double estimateX = 0, estimateY = 0;
  // Add each marker angle into the vector.
  for(auto& t : _transformations) { 
    EulerAngle e;
    convertFromMatrix(e, t.rotation().matrix());
    double theta = e.alpha();
    estimateX += std::cos(theta);
    estimateY += std::sin(theta);
  }
  estimateX /= _transformations.size();
  estimateY /= _transformations.size();
  
  // Get the estimated angle from the unit vector.
  return std::atan2(estimateY, estimateX);

}

/*----------------------------------------------------------------------------*/
