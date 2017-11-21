#include "Agent.h"

#include <iostream>
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/MPTask.h"


/*------------------------------ Construction --------------------------------*/

Agent::
Agent(Robot* const _r) : m_robot(_r) { }


Agent::
~Agent() {
  delete m_task;
}

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
  /// @WARNING Arbitrarily setting the velocity does not respect the robot's
  ///          dynamics. It is OK for now because we have not yet tried to make
  ///          the robot do anything after traveling one path. For more complex
  ///          behavior (like TMP type problems) where the robot will travel
  ///          multiple paths, this will need to be removed.
  btMultiBody* body = m_robot->GetDynamicsModel()->Get();
  body->setBaseVel({0,0,0});
  body->setBaseOmega({0,0,0});
  for(int i = 0; i < body->getNumLinks(); i++) {
    //If it's a spherical (2 dof) joint, then we must use the other version of
    // setting the link velocity dofs for each value of desired velocity.
    if(body->getLink(i).m_jointType ==
        btMultibodyLink::eFeatherstoneJointType::eSpherical) {
      btScalar temp[] = {0,0};
      body->setJointVelMultiDof(i, temp);
    }
    //Do nothing if the joint was a non-actuated joint.
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
SetCurrentTask(MPTask* const _task) {
  // Guard against re-assignment.
  if(_task == m_task)
    return;
  delete m_task;
  m_task = _task;
}


const MPTask*
Agent::
GetCurrentTask() const noexcept {
  return m_task;
}

/*----------------------------------------------------------------------------*/
