#include <exception>
#include <string>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "Utilities/PMPLExceptions.h"


int
main(int _argc, char** _argv) {
  try {
    if(_argc != 3 || std::string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    // Get the XML file name from the command line.
    std::string xmlFile = _argv[2];

    // Parse the Problem node into an MPProblem object.
    MPProblem* problem = new MPProblem(xmlFile);

    // Parse the Library node into an MPLibrary object.
    MPLibrary* pmpl = new MPLibrary(xmlFile);

    // Create storage for the solution and ask the library to solve our problem.
    Robot* const robot = problem->GetRobots().front().get();

    //solve for every task
    for(auto task : problem->GetTasks(robot))
      pmpl->Solve(problem, task.get());

    // Release resourcess.
    delete problem;
    delete pmpl;

    return 0;
  }
  catch(const std::runtime_error& _e) {
    // Write exceptions to cout so that we still get them when piping stdout.
    std::cout << std::endl << _e.what() << std::endl;
    return 1;
  }
}
