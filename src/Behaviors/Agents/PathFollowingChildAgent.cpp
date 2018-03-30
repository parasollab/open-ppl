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

#include "nonstd/container_ops.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"

#include "MPProblem/Robot/HardwareInterfaces/ArucoDetectorInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"

/*------------------------------ Construction --------------------------------*/

PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r) : PathFollowingAgent(_r) {
}


PathFollowingChildAgent::
PathFollowingChildAgent(Robot* const _r, XMLNode& _node)
  : PathFollowingAgent(_r, _node) {
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
  std::cout << m_robot->GetLabel() << " STARTING PLANNING LAZYQUERY"
            << std::endl;

  // Set the copy of this robot to virtual.
  auto currentRobot = _problem->GetRobot(m_robot->GetLabel());
  currentRobot->SetVirtual(true);

  // Create a task for the parent robot copy (because this is a shared roadmap
  // method).
  auto parentRobot = m_parentAgent->GetRobot();
  auto parentCopyRobot = _problem->GetRobot(parentRobot->GetLabel());
  auto position = m_robot->GetDynamicsModel()->GetSimulatedState();
  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, position));
  GetTask()->SetStartConstraint(std::move(start));
  GetTask()->SetRobot(parentCopyRobot);

  // Set the solution for appending with the parent copy.
  m_solution->GetPath()->Clear();
  m_solution->SetRobot(parentCopyRobot);

  // Solve for the plan.
  std::cout << "Calling Solve for " << m_robot->GetLabel()
            << "\n\tCurrently at: " << position << std::endl;

  m_library->Solve(_problem.get(), GetTask(), m_solution.get(), "LazyPRM",
      LRand(), "LazyCollisionAvoidance");

  // Reset the modified states.
  GetTask()->SetRobot(nullptr);
  m_solution->SetRobot(m_robot);
  m_library->SetMPProblem(m_robot->GetMPProblem());

  // Extract the path for this robot.
  m_pathIndex = 0;
  m_path = m_solution->GetPath()->Cfgs();

  // Compute the battery break.
  SetBatteryBreak();

  // Throw if PMPL failed to generate a solution.
  // TODO: Determine what to do when failing to produce a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

  std::cout << m_robot->GetLabel() << " DONE PLANNING LAZYQUERY" << std::endl;

  m_planning = false;
}


bool
PathFollowingChildAgent::
SelectTask(){
  m_parentAgent->AssignTask(this);
  return GetTask();
}


void
PathFollowingChildAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);

  // Update odometry tracking.
  /// @TODO This assumes we are using a velocity-based iCreate controller. Make
  ///       this explicitly required or generalize.
  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");

  // Drain the battery proportional to the controller output (higher output =
  // higher drain).
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  if(_c.size())
    m_distance += _steps * timeRes * nonstd::magnitude<double>(_c[0].GetForce());

  //std::cout << m_robot->GetLabel() << " BATTERY LEVEL: "
  //          << m_robot->GetBattery()->GetCurLevel()
  //          << std::endl;

  // Derate the emulated battery.
  // Derate faster when we are using the iCreates so that we don't need
  // to wait forever to see the switch.
  auto hardware = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base"));
  //const double depletionRate = hardware ? .36 : .04;
  const double depletionRate = hardware ? .36 : .5;
  m_robot->GetBattery()->UpdateValue(_steps * timeRes * depletionRate);
}


void
PathFollowingChildAgent::
SetBatteryBreak(){
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  // Set the battery depletion rate.
  auto hardware = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base"));
  const double depletionRate = hardware ? .36 : .5;

  const double threshold = .2 * m_robot->GetBattery()->GetMaxLevel();
  auto path = m_solution->GetPath();
  auto batteryBreak = path->FindBatteryBreak(m_robot->
                        GetBattery()->GetCurLevel(), depletionRate, threshold,
                        m_parentAgent->GetCurrentTime(), timeRes, m_library.get());
  std::cout << "Battery Break at: " << batteryBreak.GetPlace() << std::endl
            << "\t" << batteryBreak.GetTime() << std::endl;
  m_parentAgent->SetBatteryBreak(batteryBreak, this);
}

/*----------------------------------------------------------------------------*/
