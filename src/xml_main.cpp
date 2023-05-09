#include <exception>
#include <limits>
#include <string>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "Utilities/PMPLExceptions.h"


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

  // Open the XML and extract nodes.
  XMLNode mpNode(xmlFile, "MotionPlanning");
  XMLNode input(xmlFile, "Problem");
  XMLNode mpLibrary(xmlFile, "MotionPlanning");

  // Parse the Problem node into an MPProblem object.
  MPProblem* problem = new MPProblem(mpNode, input);

  size_t sl = xmlFile.rfind("/");
  auto m_filePath = xmlFile.substr(0, sl == string::npos ? 0 : sl + 1);
  problem->SetPath(m_filePath);

  // Parse the Library node into an MPLibrary object.
  XMLNode* planningLibrary = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "Library")
      planningLibrary = &child;

 MPLibrary* pmpl = new MPLibrary(*planningLibrary);

  // Create storage for the solution and ask the library to solve our problem.
  Robot* const robot = problem->GetRobots().front().get();
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

/*   // Add another task
  const std::string task_xml ="    <Task label=\"query\" robot=\"boxy\">"
      "<StartConstraints>"
          "<CSpaceConstraint point=\"-5 0 0 0 0 0\" />"
"         </StartConstraints>"
"      <GoalConstraints>"
"         <CSpaceConstraint point=\"5 1 0 0 0 0\" />"
"      </GoalConstraints>"
"    </Task>";
 
  auto task_node = XMLNode("task", task_xml, "Task");
  auto new_task = new MPTask(problem, task_node);

  // Solve Task
  pmpl->Solve(problem, new_task);
*/

  // Release resources.
  delete problem;
  delete pmpl;

  return 0;
}