#include "Agent.h"

#ifdef PMPL_SIMULATOR
//#include "BatteryConstrainedGroup.h"
#include "PathFollowingAgent.h"
#include "RoadmapFollowingAgent.h"
#endif

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Control.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/HardwareInterfaces/HardwareInterface.h"
#include "MPProblem/Robot/HardwareInterfaces/QueuedHardwareInterface.h"
#include "Utilities/XMLNode.h"

#include "nonstd/numerics.h"

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


std::unique_ptr<Agent>
Agent::
Factory(Robot* const _r, XMLNode& _node) {
  // Read the node and mark it as visited.
  std::string type = _node.Read("type", true, "", "The Agent class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<Agent> output;

#ifdef PMPL_SIMULATOR
  if(type == "pathfollowing")
    output = std::unique_ptr<PathFollowingAgent>(
        new PathFollowingAgent(_r, _node)
    );
  else if(type == "roadmapfollowing")
    output = std::unique_ptr<RoadmapFollowingAgent>(
        new RoadmapFollowingAgent(_r, _node)
    );
  //else if(type == "batteryconstrainedgroup")
  //  output = std::unique_ptr<BatteryConstrainedGroup>(
  //      new BatteryConstrainedGroup(_r, _node)
  //  );
  else
    throw ParseException(_node.Where(), "Unknown agent type '" + type + "'.");
#else
  // If we are not building the simulator, ignore the agent node.
  _node.Ignore();
#endif

  return output;
}


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
SetTask(MPTask* const _task) {
  m_task = _task;
}


MPTask*
Agent::
GetTask() const noexcept {
  return m_task;
}

/*---------------------------- Simulation Interface --------------------------*/

size_t
Agent::
NearestNumSteps(const double _dt) const {
  const double timeRes  = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes(),
               multiple = _dt / timeRes,
               threshold = timeRes * .01;

  // If _dt is approximately an integer number of time steps, return that
  // integer.
  if(nonstd::approx(multiple, std::round(multiple), threshold))
    return std::lround(multiple);
  // Otherwise, return the next largest number of steps.
  return size_t(std::ceil(multiple));
}


size_t
Agent::
MinimumSteps() const {
  // Get the problem time resolution.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  // If we are not controlling a hardware base, the problem resolution is OK.
  auto hardware = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string
  if(!hardware)
    return timeRes;

  // Otherwise, return the largest multiple of timeRes which is at least as long
  // as the hardware time.
  const double hardwareTime = hardware->GetCommunicationTime();
  return NearestNumSteps(hardwareTime);
}


std::vector<Robot*>
Agent::
ProximityCheck(const double _distance) const {
  // TODO: WeightedEuclideanDistance assumes that both robots have identical
  // configuration spaces. Adjust this function to find the true minimum
  // distance between the robot bodies.
  static WeightedEuclideanDistance<PMPLTraits> dm(1,0,0,0);
  auto problem = m_robot->GetMPProblem();

  vector<Robot*> result;

  for(auto& robotPtr : problem->GetRobots()) {
    auto robot = robotPtr.get();
    if(robot->IsVirtual() or robot == m_robot)
      continue;

    auto robotPosition = robot->GetDynamicsModel()->GetSimulatedState();
    auto myPosition = m_robot->GetDynamicsModel()->GetSimulatedState();
    double distance = dm.Distance(robotPosition, myPosition);

    if(distance < _distance){
      result.push_back(robot);
    }
  }
  return result;
}


void
Agent::
CheckRobot(const Cfg& _expected) const {
  // Find the current configuration for the robot.
  const Cfg current = m_robot->GetDynamicsModel()->GetSimulatedState();

  Cfg::PrintRobotCfgComparisonInfo(std::cout, _expected, current);
}

/*------------------------------ Agent Control -------------------------------*/

void
Agent::
Halt() {
  // Zero the robot's velocity so that we can tell that it has completed its
  // path by visual inspection.
  btMultiBody* body = m_robot->GetDynamicsModel()->Get();
  body->setBaseVel({0,0,0});
  body->setBaseOmega({0,0,0});
  for(int i = 0; i < body->getNumLinks(); i++) {
    // If it's a spherical (2 dof) joint, then we must use the other version of
    // setting the link velocity dofs for each value of desired velocity.
    if(body->getLink(i).m_jointType ==
        btMultibodyLink::eFeatherstoneJointType::eSpherical) {
      btScalar temp[] = {0,0};
      body->setJointVelMultiDof(i, temp);
    }
    // Do nothing if the joint was a non-actuated joint.
    else if (body->getLink(i).m_jointType !=
        btMultibodyLink::eFeatherstoneJointType::eFixed) {
      body->setJointVel(i, 0);
    }
  }

  if(m_debug)
    std::cout << "\nRoadmap finished."
              << "\nAll velocity DOFs set to 0 for visual inspection."
              << std::endl;
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
  return true;
}


void
Agent::
ExecuteControls(const ControlSet& _c, const size_t _steps) {
  // Store the last used controls.
  m_currentControls = _c;
  m_stepsRemaining = _steps;

  if(m_debug) {
    std::cout << "New controls selected for the next " << m_stepsRemaining
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
  auto hardware = static_cast<QueuedHardwareInterface*>
    (m_robot->GetHardwareInterface("base")); //TODO Magic string

  // Do nothing if we are not controlling a hardware base.
  if(!hardware)
    return;

  // Make sure that the requested time is at least as long as the hardware time.
  if(_steps < MinimumSteps())
    throw RunTimeException(WHERE, "Cannot enqueue command for fewer steps than "
        "the hardware's minimum response time (" + std::to_string(_steps) + " < "
        + std::to_string(MinimumSteps()) + ").");

  // Convert steps to time and enqueue the command.
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();
  hardware->EnqueueCommand(_c, _steps * timeRes);
}

/*----------------------------------------------------------------------------*/
