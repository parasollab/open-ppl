#include <exception>
#include <string>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "Utilities/PMPLExceptions.h"


int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || std::string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    std::string xmlFile = _argv[2];

    MPProblem* problem = new MPProblem(xmlFile);
    MPLibrary* pmpl = new MPLibrary(xmlFile);
    MPSolution* solution = new MPSolution;

    //pmpl->Solve(problem, problem->GetTasks().front(), solution);
    pmpl->Solve(problem, nullptr, solution);

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
