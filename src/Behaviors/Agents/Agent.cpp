#include "Agent.h"

#include <iostream>

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "MPProblem/Robot/DynamicsModel.h"
#include "MPProblem/Robot/Robot.h"



/*------------------------------ Construction --------------------------------*/

Agent::
Agent(Robot* const _r) : m_robot(_r) { }


Agent::
~Agent() = default;

/*----------------------------------------------------------------------------*/


/*------------------------------Model Affectors-------------------------------*/

void
Agent::
Halt() {
  // Zero the robot's velocity so that we can tell that it has completed its
  // path by visual inspection.
  /// @WARNING Arbitrarilly setting the velocity does not respect the robot's
  ///          dynamics. It is OK for now because we have not yet tried to make
  ///          the robot do anything after traveling one path. For more complex
  ///          behavior (like TMP type problems) where the robot will travel
  ///          multiple paths, this will need to be removed.
  m_robot->GetDynamicsModel()->Get()->setBaseVel({0,0,0});
  m_robot->GetDynamicsModel()->Get()->setBaseOmega({0,0,0});

  if(m_debug)
    std::cout << "\nRoadmap finished."
              << "\nBase velocity and omega set to 0 for visual inspection."
              << std::endl;
}

/*----------------------------------------------------------------------------*/
