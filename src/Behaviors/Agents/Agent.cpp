#include "Agent.h"

#include "Behaviors/Controllers/ControllerMethod.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/HardwareInterfaces/HardwareInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/RobotCommandQueue.h"
#include "MPProblem/Robot/HardwareInterfaces/SensorInterface.h"
#include "Simulator/Simulation.h"
#include "Utilities/PMPLExceptions.h"

#include <iostream>


/*------------------------------ Construction --------------------------------*/

Agent::
Agent(Robot* const _r) : m_robot(_r) { }


Agent::
Agent(Robot* const _r, const Agent& _a)
  : m_robot(_r),
    m_initialized(_a.m_initialized),
    m_debug(_a.m_debug)
{ }


Agent::
~Agent() = default;

/*----------------------------- Accessors ------------------------------------*/

Robot*
Agent::
GetRobot() const noexcept {
  return m_robot;
}


void
Agent::
SetTask(const std::shared_ptr<MPTask> _task) {
  m_task = _task;
}


std::shared_ptr<MPTask>
Agent::
GetTask() const noexcept {
  return m_task;
}


std::string
Agent::
GetCapability() const noexcept {
  return m_capability;
}


void
Agent::
SetCapability(std::string _capability) {
  m_capability = _capability;
}

/*------------------------------ Internal State ------------------------------*/

bool
Agent::
IsChild() const noexcept {
  return false;
}

/*---------------------------- Simulation Interface --------------------------*/

size_t
Agent::
MinimumSteps() const {
  // Get the problem time resolution.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  // If we are not controlling a hardware base, the problem resolution is OK.
  auto hardware = m_robot->GetHardwareQueue();
  if(!hardware)
    return timeRes;

  // Otherwise, return the largest multiple of timeRes which is at least as long
  // as the hardware time.
  const double hardwareTime = hardware->GetCommunicationTime();
  return Simulation::NearestNumSteps(hardwareTime);
}


std::vector<Agent*>
Agent::
ProximityCheck(const double _distance) const {
  /// @TODO This implementation checks only the distance between reference
  ///       points. Adjust this function to find the true minimum distance
  ///       between the robot bodies.
  auto problem = m_robot->GetMPProblem();
  auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState().GetPoint();

  std::vector<Agent*> result;

  for(auto& robotPtr : problem->GetRobots()) {
    auto robot = robotPtr.get();
    // Skip this agent's robot and any virtual robots.
    if(robot->IsVirtual() or robot == m_robot)
      continue;

    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState().GetPoint();
    const double distance = (robotPosition - myPosition).norm();

    if(distance < _distance){
      result.push_back(robot->GetAgent());
    }
  }
  return result;
}

/*------------------------------ Agent Control -------------------------------*/

void
Agent::
Halt() {
  btMultiBody* body = m_robot->GetDynamicsModel()->Get();

  // Zero the base velocity.
  body->setBaseVel({0,0,0});
  body->setBaseOmega({0,0,0});

  // Zero the joint velocities.
  for(int i = 0; i < body->getNumLinks(); i++) {
    // If it's a spherical (2 dof) joint, then we must use the other version of
    // setting the link velocity dofs for each value of desired velocity.
    if(body->getLink(i).m_jointType ==
        btMultibodyLink::eFeatherstoneJointType::eSpherical) {
      btScalar temp[] = {0,0};
      body->setJointVelMultiDof(i, temp);
    }
    // Do nothing if the joint was a non-actuated joint.
    else if(body->getLink(i).m_jointType !=
        btMultibodyLink::eFeatherstoneJointType::eFixed) {
      body->setJointVel(i, 0);
    }
  }

  if(m_debug)
    std::cout << "\nHalting robot '" << m_robot->GetLabel() << "'."
              << "\nAll velocity DOFs set to 0 for visual inspection."
              << std::endl;
}


void
Agent::
PauseAgent(const size_t _steps) {
  /// @TODO This isn't working quite right. Holonomic robots have no velocity,
  ///       so this mechanism doesn't work for them.
  // Set a goal at the current position with 0 velocity.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();
  Cfg desired = current;
  desired.SetLinearVelocity(Vector3d());
  desired.SetAngularVelocity(Vector3d());

  const size_t steps = std::max(_steps, MinimumSteps());
  auto emptyControl = m_robot->GetController()->operator()(current, desired,
      m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes() * steps);

  ExecuteControls({emptyControl}, steps);
}


void
Agent::
Localize() {
  // Get the hardware and sensor interface. If either is null, this agent has no
  // sensor and cannot localize.
  auto hardware = m_robot->GetHardwareQueue();
  if(!hardware)
    return;

  auto sensor = hardware->GetSensor();
  if(!sensor)
    return;

  // Pause the agent to make sure the hardware actuator doesn't continue
  // executing a previous motion command while localizing.
  PauseAgent(1);

  // Ask sensor to take measurement.
  hardware->EnqueueCommand(SensorCommand());
}


bool
Agent::
IsLocalizing() const noexcept {
  // Get the hardware and sensor interface. If either is null, this agent has no
  // sensor and cannot localize.
  auto hardware = m_robot->GetHardwareQueue();
  if(!hardware)
    return false;

  auto sensor = hardware->GetSensor();
  if(!sensor)
    return false;

  // We are localizing if the sensor isn't ready.
  return !sensor->IsReady();
}


Cfg
Agent::
EstimateState() {
  // Get odometry relative to last localization.
  // Get last sensor readings.
  // Feed into state estimator.
  // Return result.

  return Cfg();
}


bool
Agent::
ContinueLastControls() {
  // If we have no time left on the last controls, report that we are ready for
  // new ones.
  if(m_stepsRemaining == 0) {
    m_currentControls = ControlSet();
    return false;
  }

  if(m_debug)
    std::cout << "Continuing same control(s) for " << m_stepsRemaining
              << " more steps."
              << std::endl;

  --m_stepsRemaining;
  ExecuteControlsSimulation(m_currentControls);

  // TEST LOCALIZATION
  if(m_stepsRemaining == 0) {
    Localize();
  }
  return true;
}


void
Agent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  // Store the last used controls.
  m_currentControls = _c;
  m_stepsRemaining = _steps - 1;

  if(m_debug) {
    std::cout << "New controls selected for the next " << m_stepsRemaining + 1
              << " steps:";
    for(const auto& c : m_currentControls)
      std::cout << "\n\t" << c;
    std::cout << std::endl;
  }

  ExecuteControlsSimulation(_c);
  ExecuteControlsHardware(_c, _steps);
}


void
Agent::
ExecuteControlsSimulation(const ControlSet& _c) {
  // Execute the controls on the simulated robot.
  for(size_t i = 0; i < _c.size(); ++i) {
    auto& control = _c[i];
    control.Execute();

    if(m_debug)
      std::cout << "\tApplying control " << i << ": " << control
                << std::endl;
  }
}


void
Agent::
ExecuteControlsHardware(const ControlSet& _c, const size_t _steps) {
  // Do nothing if we are not controlling a hardware base.
  auto hardware = m_robot->GetHardwareQueue();
  if(!hardware)
    return;

  // Make sure that the requested time is at least as long as the hardware time.
  if(_steps < MinimumSteps())
    throw RunTimeException(WHERE) << "Cannot enqueue command for fewer steps "
                                  << "than the hardware's minimum response time "
                                  << "(" << _steps << " < " << MinimumSteps()
                                  << ").";

  // Convert steps to time and enqueue the command.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  hardware->EnqueueCommand(MotionCommand(_c, _steps * timeRes));
}

/*----------------------------------------------------------------------------*/
