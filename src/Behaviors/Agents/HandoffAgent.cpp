#include "HandoffAgent.h"

#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Behaviors/Controllers/ICreateController.h"
#include "Behaviors/Agents/Coordinator.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/Constraints/BoundaryConstraint.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletModel.h"
#include "Simulator/MicroSimulator.h"
#include "Simulator/Simulation.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/container_ops.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include <limits>
#include <unordered_map>
#include <algorithm>


/*------------------------------ Construction --------------------------------*/

HandoffAgent::
HandoffAgent(Robot* const _r) : PathFollowingAgent(_r) {
}


HandoffAgent::
HandoffAgent(Robot* const _r, XMLNode& _node)
  : PathFollowingAgent(_r, _node) {
  // Parse XML parameters.
}


HandoffAgent::
~HandoffAgent() {
  // Ensure agent is properly torn down.
  Uninitialize();
}

/*---------------------------- Simulation Interface --------------------------*/

void
HandoffAgent::
Initialize() {
  PathFollowingAgent::Initialize();
}

void
HandoffAgent::
InitializeRoadmap() {
  m_solution = std::unique_ptr<MPSolution>(new MPSolution(m_robot));
}

Coordinator*
HandoffAgent::
GetParentAgent(){
  return m_parentAgent;
}

void
HandoffAgent::
SetParentAgent(Coordinator* const _parent) {
  m_parentAgent = _parent;
}


bool
HandoffAgent::
IsChild() const noexcept {
  return true;
}


void
HandoffAgent::
GenerateCost(std::shared_ptr<MPTask> const _task) {
  std::cout << "Starting generate cost function" << std::endl;
  // Save current state in case the robot has not finished its current task.
  auto currentTask = GetTask();
  auto currentPath = m_path;

  Cfg currentPos = m_robot->GetSimulationModel()->GetState();
  // TODO: Create cost functions for the path (adjust edge weights according to
  // metric and agent type).
  m_potentialCost = 0.0;

  if(!currentPath.empty())
    m_potentialCost = m_solution->GetPath()->Length();

  // Compute cost to travel from current position to start constraint
  Cfg position;
  // If the robot currently has a path, set its position to the end of that path
  // Otherwise, the robot will start the task from its current position
  if(!currentPath.empty())
    position = m_path.back();
  else
    position = m_robot->GetSimulationModel()->GetState();

  std::shared_ptr<MPTask> setupTask(new MPTask(m_robot));
  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, position));


  setupTask->SetStartConstraint(std::move(start));
  std::unique_ptr<Constraint> setupStart(_task->GetStartConstraint()->Clone());

  if(m_parentAgent->GetRobot()->IsManipulator()){

    auto startBox = setupStart->GetBoundary()->Clone();
    auto box = static_cast<CSpaceBoundingBox*>(startBox.get());
    auto ranges = box->GetRanges();
    for(size_t i = 3; i < ranges.size(); i++){
      box->SetRange(i, -1, 1);
    }

    std::cout << "Ranges for start constraint" << std::endl;
    for(auto r : box->GetRanges()){
      std::cout << r << std::endl;
    }

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(startBox)));
    if(!startConstraint->Satisfied(position)){
      m_potentialCost = std::numeric_limits<size_t>::max();
      currentPos.ConfigureRobot();
      return;
    }

  }

  setupTask->AddGoalConstraint(std::move(setupStart));
  SetTask(setupTask);
  std::shared_ptr<MPProblem> problemCopy(new MPProblem(*m_robot->GetMPProblem()));
  try{
    WorkFunction(problemCopy);
    if(!m_solution->GetPath()->Cfgs().empty()){
      auto setupPath = m_path;
      m_potentialCost += m_solution->GetPath()->Length();

      // Compute cost to travel from start constraint to goal constraint
      SetTask(_task);
      std::shared_ptr<MPProblem> problemCopyDos(new MPProblem(*m_robot->GetMPProblem()));
      WorkFunction(problemCopyDos);

      if(!m_solution->GetPath()->Cfgs().empty()){
        auto goalPath = m_path;
        m_potentialCost += m_solution->GetPath()->Length();

        // Save the computed paths in case this robot is selected to perform the task.
        m_potentialPath = currentPath;
        m_potentialPath.insert(m_potentialPath.end(), setupPath.begin(), setupPath.end());
        m_potentialPath.insert(m_potentialPath.end(), goalPath.begin(), goalPath.end());


        const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
        auto controller  = m_robot->GetController();
        auto dynamics    = m_robot->GetMicroSimulator();
        auto dm          = m_library->GetDistanceMetric(m_waypointDm);

        double numSteps = 0;

        for(size_t i = 1; i < m_potentialPath.size(); ++i) {
          // Get the next pair of configurations.
          Cfg         current  = m_potentialPath[i - 1];
          const auto& waypoint = m_potentialPath[i];

          // While current is too far from way point, use the controller to generate
          // a control and test it with the dynamics model.
          while(dm->Distance(current, waypoint) > m_waypointThreshold) {
            // Apply the next control.
            Control nextControl = (*controller)(current, waypoint, timeRes);
            current = dynamics->Test(current, nextControl, timeRes);
            numSteps += 1;
          }
        }
        double time = numSteps*timeRes;
        double currentTime = m_parentAgent->GetCurrentTime();
        m_potentialCost = time + currentTime;

      }
      else{ // Robot cannot complete the task
        m_potentialCost = std::numeric_limits<size_t>::max();
      }
    }
    else { // Robot cannot get to the start location of the task
      m_potentialCost = std::numeric_limits<size_t>::max();
    }
  }
  catch(...){
    m_potentialCost = std::numeric_limits<size_t>::max();
  }
  // Restore the task/path state to currentTask/currentPath
  SetTask(currentTask);
  //Need to restore path because it is over written in the Work Function
  m_path = currentPath;
  currentPos.ConfigureRobot();
  std::cout << "Finishing generate cost function" << std::endl;
}

double
HandoffAgent::
GetPotentialCost() const {
  return m_potentialCost;
}


double
HandoffAgent::
GetTaskTime() const {
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  // Use the controller and dynamics model to generate an ideal course for this
  // path.
  const auto& path = m_solution->GetPath()->Cfgs();
  auto controller  = m_robot->GetController();
  auto dynamics    = m_robot->GetMicroSimulator();
  auto dm          = m_library->GetDistanceMetric(m_waypointDm);

  double numSteps = 0;

  for(size_t i = 1; i < path.size(); ++i) {
    // Get the next pair of configurations.
    Cfg         current  = path[i - 1];
    const auto& waypoint = path[i];

    // While current is too far from way point, use the controller to generate
    // a control and test it with the dynamics model.
    while(dm->Distance(current, waypoint) > m_waypointThreshold) {

      // Apply the next control.
      Control nextControl = (*controller)(current, waypoint, timeRes);
      current = dynamics->Test(current, nextControl, timeRes);
      numSteps += 1;
    }
  }

  return numSteps * timeRes;

}
/*------------------------------ Helpers -------------------------------------*/

void
HandoffAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  std::cout << "GENERATING NEW PLAN" << std::endl;
  // TODO: Stop trying to plan if it takes longer than t_max
  // TODO: Parameterize this later to avoid hardcoding to LazyPRM
  std::cout << m_robot->GetLabel()
            << " STARTING PLANNING LAZYQUERY"
            << std::endl;

  for(auto& robot : _problem->GetRobots()){
    if(robot->GetLabel() == m_parentAgent->GetRobot()->GetLabel())
      robot->SetVirtual(true);
    else{
      robot->SetVirtual(false);
    }
  }
  m_robot->SetVirtual(true);
  // Create a task for the parent robot copy (because this is a shared roadmap
  // method).

  //auto copyRobot = _problem->GetRobot(m_robot->GetLabel());

  //GetTask()->SetRobot(copyRobot);
  GetTask()->SetRobot(m_robot);
  std::cout << "Calling Solve for " << m_robot->GetLabel() <<  std::endl;
  std::cout << "Currently at: " << m_robot->GetSimulationModel()->GetState()
            << std::endl;
  if(m_solution){
    m_solution->GetPath()->Clear();
  }
  else {
    m_solution = unique_ptr<MPSolution>(new MPSolution(m_robot));
    //m_solution = m_library->GetMPSolution();
  }
  // Set the solution for appending with the parent copy.
  //m_solution->SetRobot(copyRobot);
  m_solution->SetRobot(m_robot);

  if(m_debug){
    m_graphVisualID = Simulation::Get()->AddRoadmap(m_solution->GetRoadmap()->GetGraph(),
      glutils::color(0., 1., 0., 0.2));
  }
  // Solve for the plan.
  std::cout << "Calling Solve for " << m_robot->GetLabel() << std::endl;

  if(!m_parentAgent->GetRobot()->IsManipulator()){
    //m_library->Solve(_problem.get(), GetTask().get(), m_solution.get(),
    //    "LazyPRM", LRand(), "LazyCollisionAvoidance");
    m_library->Solve(_problem.get(), GetTask().get(), m_solution.get(),
        "EvaluateMapStrategy", LRand(), "LazyCollisionAvoidance");
  }
  else {
    auto startBox = GetTask()->GetStartConstraint()->GetBoundary()->Clone();
    auto box = static_cast<CSpaceBoundingBox*>(startBox.get());
    auto ranges = box->GetRanges();
    for(size_t i = 0; i < 3; i++){
      box->SetRange(i, box->GetRange(i).min-.005, box->GetRange(i).max+.005);
    }
    for(size_t i = 3; i < ranges.size(); i++){
      box->SetRange(i, box->GetRange(i).min-.05, box->GetRange(i).max+.05);
    }

    std::cout << "Ranges for start constraint" << std::endl;
    for(auto r : box->GetRanges()){
      std::cout << r << std::endl;
    }

    auto startConstraint = std::unique_ptr<BoundaryConstraint>
      (new BoundaryConstraint(m_robot, std::move(startBox)));
    auto g = m_solution->GetRoadmap(m_robot)->GetGraph();
    for(auto vit = g->begin(); vit != g->end(); vit++){
      std::cout << vit->property().PrettyPrint() << " : "
                << vit->descriptor() << " : "
                << vit->property().GetRobot() << std::endl;
      auto start = GetTask()->GetStartConstraint();
      std::cout << "Satisfied: " << start->Satisfied(vit->property()) << std::endl;
    }

    std::cout << g->GetVertex(0).PrettyPrint();
    std::cout << g->GetVertex(1).PrettyPrint();

    m_library->Solve(_problem.get(), GetTask().get(), m_solution.get(),
        "EvaluateMapStrategy", LRand(), "LazyCollisionAvoidance");
  }

  // Reset the modified states.
  GetTask()->SetRobot(m_robot);
  m_solution->SetRobot(m_robot);

  // Extract the path for this robot.
  m_pathIndex = 0;
  m_path = m_solution->GetPath()->Cfgs();
  // Throw if PMPL failed to generate a solution.
  // TODO: Determine what to do when failing to produce a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

  std::cout << "Printing out full path" << std::endl;
  for(auto cfg : m_solution->GetPath()->FullCfgs(m_library.get(), "slRobot")){
    std::cout << cfg.PrettyPrint() << std::endl;
  }

  std::cout << m_robot->GetLabel() << " DONE PLANNING LAZYQUERY" << std::endl;

  m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
  m_planning = false;
}

bool
HandoffAgent::
SelectTask(){
  if(GetTask().get())
    return true;
  if(m_queuedSubtasks.size() == 0){
    m_priority = 0;
    return false;
  }

  auto subtask = m_queuedSubtasks.front();
  subtask->SetRobot(m_robot);
  auto startConstraint = subtask->GetStartConstraint();

  auto pos = m_robot->GetSimulationModel()->GetState();
  std::cout << pos.PrettyPrint() << std::endl;

  if(m_parentAgent->GetRobot()->IsManipulator()){

    auto startBox = startConstraint->GetBoundary()->Clone();
    auto box = static_cast<CSpaceBoundingBox*>(startBox.get());
    auto ranges = box->GetRanges();
    for(size_t i = 0; i < 3; i++){
      auto range = ranges[i];
      auto center = range.Center();
      box->SetRange(i, center-.05, center+.05);
    }

    std::cout << "Ranges for start constraint" << std::endl;
    for(auto r : box->GetRanges()){
      std::cout << r << std::endl;
    }

    startConstraint = new BoundaryConstraint(m_robot, std::move(startBox));

  }

  if(m_debug){
    std::cout << "CHECKING IF START CONSTRAINT IS SATISFIED: " << m_robot->GetLabel() << std::endl;
    std::cout << "cfg data: " << pos.GetData() << std::endl;
  }
  if(!startConstraint->Satisfied(pos)){
    if(m_debug){
      std::cout << "Not satisfied" << std::endl;
      std::cout << "Generating Setup task for: " << m_robot->GetLabel() << std::endl;
    }

    std::shared_ptr<MPTask> setupTask = std::shared_ptr<MPTask>(new MPTask(m_robot));
    std::unique_ptr<CSpaceConstraint> start = std::unique_ptr<CSpaceConstraint>(
                                              new CSpaceConstraint(m_robot, pos));

    setupTask->SetStartConstraint(std::move(start));

    std::unique_ptr<Constraint> goal = startConstraint->Clone();

    setupTask->AddGoalConstraint(std::move(goal));

    this->SetTask(setupTask);
    m_performingSubtask = false;
    m_priority = 500;
    return GetTask().get();
  }
  else {
    if(m_debug){
      std::cout << "Satisfied" << std::endl;
      std::cout << "Starting next subtask for: " << m_robot->GetLabel() << std::endl;
    }
    m_clearToMove = false;
    m_performingSubtask = true;
    this->SetTask(m_queuedSubtasks.front());
    m_queuedSubtasks.pop_front();
    m_priority = 1000;
    return GetTask().get();
  }
}

bool
HandoffAgent::
EvaluateTask(){
  auto task = GetTask();
  if(!this->PathFollowingAgent::EvaluateTask()){
    if(m_debug){
      std::cout << "Performing Subtask: " << m_performingSubtask << std::endl;
      std::cout << "I, " << m_robot->GetLabel() << " HAVE COMPLETED MY TASK ";
    }
    SetTask(task);
    SetPriority(0);
    if(m_clearToMove or m_parentAgent->IsClearToMoveOn(this)){
      if(m_debug){
        std::cout << "AND I AM CLEAR TO MOVE ON";
      }
      SetTask(nullptr);

      m_clearToMove = false;

      return false;
    }
    else {
      if(m_debug){
        std::cout << "AND I AM NOTTTTTTTT CLEAR TO MOVE ON";
      }
      m_path.push_back(m_robot->GetSimulationModel()->GetState());
    }
  }
  return true;

}

bool
HandoffAgent::
ReachedHandoff(){
  if(m_clearToMove){
    return true;
  }
  return !PathFollowingAgent::EvaluateTask();
}

void
HandoffAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);
  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");

  // Update odometry tracking.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  if(_c.size())
    m_distance += _steps * timeRes * nonstd::magnitude<double>(_c[0].GetOutput());
}

MPSolution*
HandoffAgent::
GetMPSolution(){
  return m_solution.get();
}

void
HandoffAgent::
SetRoadmapGraph(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph){
  m_solution->GetRoadmap(m_robot)->SetGraph(_graph);
}

void
HandoffAgent::
AddSubtask(std::shared_ptr<MPTask> _task){
  m_queuedSubtasks.push_back(_task);
}


std::list<std::shared_ptr<MPTask>>
HandoffAgent::
GetQueuedSubtasks(){
  return m_queuedSubtasks;
}

bool
HandoffAgent::
IsPerformingSubtask(){
  return m_performingSubtask;
}

size_t
HandoffAgent::
GetPriority(){
  return m_priority;
}

void
HandoffAgent::
SetPriority(size_t _p){
  m_priority = _p;
}

void
HandoffAgent::
SetPerformingSubtask(bool _performing){
  m_performingSubtask = _performing;
}

void
HandoffAgent::
SetClearToMove(bool _clear){
  m_clearToMove = _clear;
}
