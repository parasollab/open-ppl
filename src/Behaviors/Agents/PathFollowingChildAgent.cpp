#include "PathFollowingChildAgent.h"

#include <limits>
#include <unordered_map>
#include <algorithm>
#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/BatteryConstrainedGroup.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "MPProblem/Robot/HardwareInterfaces/ArucoDetectorInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"

/*------------------------------ Construction --------------------------------*/

PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r) : PathFollowingAgent(_r) {
}


PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r, XMLNode& _node) : PathFollowingAgent(_r, _node) {
  // Parse XML parameters.
}


PathFollowingChildAgent::
~PathFollowingChildAgent() {
  // Ensure agent is properly torn down.
  Uninitialize();
}

/*----------------------------- Child Interface ------------------------------*/

bool
PathFollowingChildAgent::
IsBatteryLow() {
  auto battery = m_robot->GetBattery();
  const double threshold = .2 * battery->GetMaxLevel();
  return battery->GetCurLevel() < threshold;
}


bool
PathFollowingChildAgent::
IsBatteryHigh() {
  auto battery = m_robot->GetBattery();
  const double threshold = .9 * battery->GetMaxLevel();
  return battery->GetCurLevel() >= threshold;
}

/*--------------------------- Internal State ---------------------------------*/

void
PathFollowingChildAgent::
SetParentAgent(BatteryConstrainedGroup* const _parent) {
  m_parentAgent = _parent;  
}

bool
PathFollowingChildAgent::
IsChild() const {
  return true;
}


/*------------------------------ Helpers -------------------------------------*/
void
PathFollowingChildAgent::
Rotate(double& _angle, double _dt) {

  Cfg point(m_robot);
  point.SetData(std::vector<double>{0, 0, 0, 0, 0, 0});

  //TODO: Hacky solution to make it rotate. Could be done in a better way,
  //maybe?
  Cfg point2(m_robot);
  point2.SetData(std::vector<double>{0, 0, _angle / M_PI, 0, 0, 0});

  auto bestControl = m_robot->GetController()->operator()
    (point2, point, _dt);

  // Execute the control.
  const size_t steps = std::max(NearestNumSteps(_dt), MinimumSteps());
  
  ExecuteControls({bestControl}, steps);
  vector<double> temp = bestControl.GetForce();
  _angle -= abs(temp[2]*_dt);
}

void
PathFollowingChildAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  // TODO: Stop trying to plan if it takes longer than t_max
  // TODO: Parameterize this later to avoid hardcoding to LazyPRM
  std::cout << m_robot->GetLabel() << " STARTING PLANNING LAZYQUERY" << std::endl;
  
  // Set the robot being planned for to virtual.
  auto currentRobot = _problem->GetRobot(m_robot->GetLabel());
  currentRobot->SetVirtual(true);
  
  // Set the task robot to the parent COPY because this is a shared roadmap method.
  auto parentRobot = m_parentAgent->GetRobot();
  auto parentCopyRobot = _problem->GetRobot(parentRobot->GetLabel());
  GetTask()->SetRobot(parentCopyRobot);

  m_library->Solve(_problem.get(), GetTask(), m_solution.get(), "LazyPRM",
      LRand(), "LazyCollisionAvoidance");
  m_planning = false;

  currentRobot->SetVirtual(false);
  GetTask()->SetRobot(nullptr);

  // Set m_path so HasPlan() returns true.
  m_path.clear();
  auto path = m_solution->GetPath()->Cfgs();
  for(auto& cfg : path) {
    m_path.emplace_back(m_robot);
    m_path.back().SetData(cfg.GetData());
  }

  m_pathIndex = 0;

  // Throw if PMPL failed to generate a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");
  std::cout << m_robot->GetLabel() << " DONE PLANNING LAZYQUERY" << std::endl;
}

bool 
PathFollowingChildAgent::
SelectTask(){
  m_parentAgent->AssignTask(this);
  return GetTask();
}

bool
PathFollowingChildAgent::
EvaluateTask(){
  if(m_debug)
    std::cout << "Evaluating task progress:";

//  auto task = GetTask();
//  const Cfg robotPos = m_robot->GetDynamicsModel()->GetSimulatedState();
//  auto status = task->Evaluate({robotPos});
//  // TODO: Make sure that the task is evaluating correctly.
//  if(status == MPTask::Status::Complete){
//    if(m_debug)
//      std::cout << "\n\tTask completed." << std::endl;
//    // If the agent completed its task, mark task as complete and return false
//    task->SetCompleted();
//    SetTask(nullptr);
//    ClearPlan();
//    return false;
//  }
  // If the battery is low, clear the task.
  if(IsBatteryLow()){
    if(m_debug)
      std::cout << "\n\tBattery low, aborting task." << std::endl;
    // TODO Maybe add this task to the list of paused tasks? Make sure this
    // makes sense with the BatteryConstrainedGroup.
    SetTask(nullptr);
    ClearPlan();
    return false;
  }
  // Agent should continue task (no problems)
  else
    return PathFollowingAgent::EvaluateTask();
}

void
PathFollowingChildAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {

  this->Agent::ExecuteControls(_c, _steps);

  // Update odometry tracking.
  /// @TODO This assumes we are using a velocity-based iCreate controller. Make
  ///       this explicitly required or generalize.
  /// @TODO This assumes that we are only passing in one control.
  if(_c.size() > 1)
    throw RunTimeException(WHERE, 
          "We are assuming that only one control will be passed in at a time.");

  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes(); 

  if(_c.size())
    m_distance += _steps * timeRes * _c[0].GetForce()[0];

  // Derate the emulated battery.
  // Derate faster when we are using the iCreates so that we don't need
  // to wait forever to see the switch.
  auto hardware = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base"));
  const double depletionRate = hardware ? .36 : .04;
  m_robot->GetBattery()->UpdateValue(_steps * timeRes * depletionRate);

}
/*----------------------------------------------------------------------------*/
