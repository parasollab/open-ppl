#include <exception>
#include <limits>
#include <string>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "Utilities/PMPLExceptions.h"

#include "MPProblem/Robot/KDLModel.h"

int
main(int _argc, char** _argv) {
  // Assert that this platform supports an infinity for doubles.
  if(!std::numeric_limits<double>::has_infinity)
    throw RunTimeException(WHERE) << "This platform does not support infinity "
                                  << "for double-types, which is required for "
                                  << "pmpl to work properly.";

  if(_argc != 3 || std::string(_argv[1]) != "-f")
    throw ParseException(WHERE) << "Incorrect usage. Usage: -f options.xml";

  // Get the XML file name from the command line.
  std::string xmlFile = _argv[2];

  // Parse the Problem node into an MPProblem object.
  MPProblem* problem = new MPProblem(xmlFile);

  // Parse the Library node into an MPLibrary object.
  MPLibrary* pmpl = new MPLibrary(xmlFile);

  // Create storage for the solution and ask the library to solve our problem.
  Robot* const robot = problem->GetRobots().front().get();

  // Temp stuff to test IK
  auto kdl = robot->GetKDLModel();
  std::vector<double> pos = {.3,-.3,.5};
  std::vector<std::vector<double>> ori = {{1,0,0},{0,1,0},{0,0,1}};

  auto joints = kdl->InverseKinematics(pos,ori);

  const auto robotTasks = problem->GetTasks(robot);
    for(auto task : robotTasks)
      if(!task->GetStatus().is_complete())
      	pmpl->Solve(problem, task.get());

  // Also solve the group task(s).
  if(!problem->GetRobotGroups().empty()) {
    RobotGroup* const robotGroup = problem->GetRobotGroups().front().get();
    for(auto groupTask : problem->GetTasks(robotGroup))
      pmpl->Solve(problem, groupTask.get());
  }

  if(robotTasks.empty() and (problem->GetRobotGroups().empty() or
      problem->GetTasks(problem->GetRobotGroups().front().get()).empty()))
    throw RunTimeException(WHERE) << "No tasks were specified!";

  // Release resources.
  delete problem;
  delete pmpl;

  return 0;
}
