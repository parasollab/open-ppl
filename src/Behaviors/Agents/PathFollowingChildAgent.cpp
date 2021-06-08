#include "PathFollowingChildAgent.h"

#include <limits>
#include <unordered_map>
#include <algorithm>
#include "ConfigurationSpace/Cfg.h"
#include "Behaviors/Agents/BatteryConstrainedGroup.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "Behaviors/Controllers/ICreateController.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/HardwareInterfaces/Battery.h"
#include "Simulator/BulletModel.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/container_ops.h"
#include "nonstd/numerics.h"
#include "nonstd/timer.h"
#include "nonstd/io.h"

#include "Simulator/Simulation.h"

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

/*---------------------------- Simulation Interface --------------------------*/

void
PathFollowingChildAgent::
Initialize() {
  PathFollowingAgent::Initialize();

  // This agent assumes we are using a velocity-based iCreate controller. Assert
  // this now.
  auto controller = m_robot->GetController();
  ICreateController* test = dynamic_cast<ICreateController*>(controller);
  if(!test)
    throw RunTimeException(WHERE, "The robot must use an ICreateController "
        "with this agent.");

  const auto& actuators = m_robot->GetActuators();
  for(const auto& pair : actuators) {
    const auto& actuator = pair.second;
    if(actuator->GetDynamicsType() == Actuator::DynamicsType::Force)
      throw RunTimeException(WHERE, "The robot must use velocity actuators "
          "with this agent.");
  }
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
IsChild() const noexcept {
  return true;
}

/*------------------------------ Helpers -------------------------------------*/

void
PathFollowingChildAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  //if(!m_parentAgent->ClearToPlan(this))
  //  return;
  // TODO: Stop trying to plan if it takes longer than t_max
  // TODO: Parameterize this later to avoid hardcoding to LazyPRM
  std::cout << m_robot->GetLabel()
            << " STARTING PLANNING LAZYQUERY"
            << std::endl;
  std::cout << "Battery level: "
            << m_robot->GetBattery()->GetCurLevel()
            << std::endl;
  // Set the copy of this robot to virtual.
  auto currentRobot = _problem->GetRobot(m_robot->GetLabel());
  currentRobot->SetVirtual(true);

  // Create a task for the parent robot copy (because this is a shared roadmap
  // method).
  auto parentRobot = m_parentAgent->GetRobot();
  auto parentCopyRobot = _problem->GetRobot(parentRobot->GetLabel());
  auto position = m_robot->GetSimulationModel()->GetState();
  auto start = std::unique_ptr<CSpaceConstraint>(
      new CSpaceConstraint(m_robot, position));
  GetTask()->SetStartConstraint(std::move(start));
  GetTask()->SetRobot(parentCopyRobot);
  std::cout << "Calling Solve for " << m_robot->GetLabel() <<  std::endl;
  std::cout << "Currently at: " << m_robot->GetSimulationModel()->GetState() << std::endl;
  m_solution->GetPath()->Clear();
  // Set the solution for appending with the parent copy.
  m_solution->SetRobot(parentCopyRobot);

  // Solve for the plan.
  std::cout << "Calling Solve for " << m_robot->GetLabel()
            << "\n\tCurrently at: " << position << std::endl;

  m_library->Solve(_problem.get(), GetTask().get(), m_solution.get(), "LazyPRM",
      LRand(), "LazyCollisionAvoidance");

  // Reset the modified states.
  GetTask()->SetRobot(m_robot);
  m_solution->SetRobot(m_robot);

  // Extract the path for this robot.
  m_pathIndex = 0;
  m_path = m_solution->GetPath()->Cfgs();

  std::cout << m_path << std::endl;

  // Throw if PMPL failed to generate a solution.
  // TODO: Determine what to do when failing to produce a solution.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

  std::cout << m_robot->GetLabel() << " DONE PLANNING LAZYQUERY" << std::endl;

  // Compute the battery break.
  SetBatteryBreak();

  //std::cout << "Waiting 100 steps: " << std::endl;
  //PauseAgent(100);
  m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
  m_planning = false;
}

bool
PathFollowingChildAgent::
SelectTask() {
  m_parentAgent->AssignTask(this);
  return GetTask().get();
}

void
PathFollowingChildAgent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  this->Agent::ExecuteControls(_c, _steps);

  if(_c.size() > 1)
    throw RunTimeException(WHERE,
        "We are assuming that only one control will be passed in at a time.");

  // Update odometry tracking.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  if(_c.size())
    m_distance += _steps * timeRes * nonstd::magnitude<double>(_c[0].GetOutput());

  // TODO Drain the battery proportional to the controller output (higher output
  // = higher drain).
  //std::cout << m_robot->GetLabel() << " BATTERY LEVEL: "
  //          << m_robot->GetBattery()->GetCurLevel()
  //          << std::endl;

  // Derate the emulated battery.
  // Derate faster when we are using the iCreates so that we don't need
  // to wait forever to see the switch.
  //auto hardware = m_robot->GetHardwareQueue();
  //const double depletionRate = hardware ? .36 : .04;
  //const double depletionRate = hardware ? .36 : .5;
  //m_robot->GetBattery()->UpdateValue(_steps * timeRes * depletionRate);
}


void
PathFollowingChildAgent::
SetBatteryBreak() {
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  // Set the battery depletion rate.
  auto hardware = m_robot->GetHardwareQueue();
  const double depletionRate = hardware ? .36 : .5;

  const double batteryThreshold = .18 * m_robot->GetBattery()->GetMaxLevel();
  double batteryLevel = m_robot->GetBattery()->GetCurLevel();

  std::cout << "Initial battery level: " << batteryLevel << std::endl;
  std::cout << "Battery Threshold: " << batteryThreshold << std::endl;

  // Use the controller and dynamics model to generate an ideal course for this
  // path.
  const auto& path = m_solution->GetPath()->Cfgs();
  auto controller  = m_robot->GetController();
  auto dm          = m_library->GetDistanceMetric(m_waypointDm);

  double numSteps = 0;

  for(size_t i = 1; i < path.size(); ++i) {
    // Get the next pair of configurations.
    Cfg         current  = path[i - 1];
    const auto& waypoint = path[i];

    // While current is too far from way point, use the controller to generate
    // a control and test it with the dynamics model.
    while(dm->Distance(current, waypoint) > m_waypointThreshold) {
      // If the battery level is too low for another step, there is a break now.
      if(batteryLevel - depletionRate < batteryThreshold) {
        BatteryBreak batteryBreak(current,
            m_parentAgent->GetCurrentTime() + numSteps * timeRes);

        std::cout << "Battery Break at: " << batteryBreak.GetPlace()
                  << "\n\t" << batteryBreak.GetTime()
                  << std::endl;

        m_parentAgent->SetBatteryBreak(batteryBreak, this);
        return;
      }

      // Otherwise, apply the next control.
      Control nextControl = (*controller)(current, waypoint, timeRes);
      current = m_robot->GetMicroSimulator()->Test(current, nextControl, timeRes);
      numSteps += 1;
      batteryLevel -= depletionRate * timeRes;
    }
  }

  double departureTime = GetTask()->GetEstimatedCompletionTime() - numSteps*timeRes;
  double currentTime =m_parentAgent->GetCurrentTime();
  if(departureTime > currentTime) {
    m_stepsRemaining = std::ceil((departureTime - currentTime)/timeRes);
  }
  if(batteryLevel - batteryThreshold < 1.5) {
    BatteryBreak batteryBreak(path.back(), m_parentAgent->GetCurrentTime() + numSteps * timeRes);

    std::cout << "Battery Break at: " << batteryBreak.GetPlace()
      << "\n\t" << batteryBreak.GetTime()
      << std::endl;

    m_parentAgent->SetBatteryBreak(batteryBreak, this);

  }

  //auto batteryBreak = path->FindBatteryBreak(batteryLevel, depletionRate, batteryThreshold,
  //                      m_parentAgent->GetCurrentTime(), timeRes, m_library.get());
  //std::cout << "Battery Break at: " << batteryBreak.GetPlace() << std::endl
  //          << "\t" << batteryBreak.GetTime() << std::endl;

  //m_parentAgent->SetBatteryBreak(batteryBreak, this);
}

void
PathFollowingChildAgent::
SetPausedTask(std::shared_ptr<MPTask> _pausedTask) {
  m_pausedTask = _pausedTask;
}

std::shared_ptr<MPTask>
PathFollowingChildAgent::
GetPausedTask() {
  return m_pausedTask;
}

/*----------------------------------------------------------------------------*/
