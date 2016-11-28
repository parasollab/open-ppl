#include "MPLibrary/PMPL.h"

#include <string>
#include <exception>

int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || std::string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    MPProblem* problem = new MPProblem(_argv[2]);
    MPLibrary* pmpl = new MPLibrary(_argv[2]);
    MPTask* task = nullptr;
    MPSolution* solution = new MPSolution;
    pmpl->Solve(problem, task, solution);

    delete problem;
    delete task;
    delete solution;
    delete pmpl;

    return 0;
  }
  catch(const std::runtime_error& e) {
    std::cerr << std::endl << e.what() << std::endl;
    return 1;
  }
}
