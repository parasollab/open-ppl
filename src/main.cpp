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
    /// @TODO Generalize this to handle more than just the first task.
    MPTask* task = problem->GetTasks().front();
    MPSolution* solution = new MPSolution(task->GetRobot());
    pmpl->Solve(problem, task, solution);

    // Release resources.
    delete problem;
    delete solution;
    delete pmpl;

    return 0;
  }
  catch(const std::runtime_error& e) {
    std::cerr << std::endl << e.what() << std::endl;
    return 1;
  }
}
