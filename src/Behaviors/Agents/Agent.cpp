#include "Agent.h"

#ifdef PMPL_SIMULATOR
//#include "BatteryConstrainedGroup.h"
#include "PathFollowingAgent.h"
#include "RoadmapFollowingAgent.h"
#endif

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/XMLNode.h"
#include "MPLibrary/DistanceMetrics/WeightedEuclideanDistance.h"
#include "MPProblem/MPProblem.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPLibrary/PMPL.h"

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
  else if(type == "batteryconstrainedgroup")
    output = std::unique_ptr<BatteryConstrainedGroup>(
        new BatteryConstrainedGroup(_r, _node)
    );
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

/*---------------------------- Simulation Interface --------------------------*/

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

/*----------------------------------------------------------------------------*/
