#include "PathFollowingAgent.h"

#include "MPProblem/ConfigurationSpace/Cfg.h"
#include "Traits/CfgTraits.h"
typedef MPTraits<Cfg> PMPLTraits;
typedef PMPLTraits::MPLibraryType MPLibraryType;


/*------------------------------ Construction --------------------------------*/

PathFollowingAgent::
PathFollowingAgent(Robot* const _r) : Agent(_r) { }

PathFollowingAgent::
~PathFollowingAgent() = default;

/*------------------------------ Agent Interface -----------------------------*/

void
PathFollowingAgent::
Initialize() {
  /// @TODO Need to have access to the problem through the robot!
  //auto problem = m_robot->GetMPProblem();

  /// @TODO Do not hard-code this - get it from the problem.
  const std::string xmlFile = "Examples/CfgExamples.xml";

  /// @TODO Call pmpl when we can access the problem
  //MPLibraryType* pmpl = new MPLibraryType(xmlFile);
  //pmpl->SetMPProblem(problem);
  //pmpl->Solve();

  /// @TODO Extract the path from the problem

  delete pmpl;
}


void
PathFollowingAgent::
Step() {
  /// @TODO Tell controller to follow path points.
}


void
PathFollowingAgent::
Uninitialize() {
  /// @TODO clear the path.
}

/*----------------------------------------------------------------------------*/
