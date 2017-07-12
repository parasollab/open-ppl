#include "ICreateAgent.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"

#include "RobotController.h"


ICreateAgent::
ICreateAgent(Robot* const _r, const std::string& _ip)
  : RoadmapFollowingAgent(_r), m_ip(_ip) {}


void ICreateAgent::ApplyCurrentControls() {
  // Assert that the robot type makes sense.
  if(m_robot->GetMultiBody()->DOF() != 3)
    throw RunTimeException(WHERE, "This only works for creates which are 3DOF "
        "planar translational.");

  // Ensure we have a create controller.
  static RobotController createController(m_ip);
  createController.StartCommandQueue(); // Does nothing after first call.

  // Apply the controls to the simulated robot.
  RoadmapFollowingAgent::ApplyCurrentControls();

  // Apply the controls to the iCreate.

  // Determine the translation and rotation signals.
  const ControlSet& controls = m_edge->GetControlSet();
  if(controls.size() != 1)
    throw RunTimeException(WHERE, "Multiple controls are not supported.");

  const Control::Signal& s = controls.front().signal;
  const double translation = s[0] / 2.,
               rotation    = s[2] / 2.;

  // Determine the length of time to execute the controls.
  const double time = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes();

  createController.EnqueueCommand(translation, rotation, time);
}
