#include "MPLibrary/PMPL.h"

#include <string>
#include <exception>

int
main(int _argc, char** _argv) {
  try {
    if(_argc < 3 || std::string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    std::string xmlFile = _argv[2];

    MPProblem* problem = new MPProblem(xmlFile);
    MPLibrary* pmpl = new MPLibrary(xmlFile);

    // Manually set up single-task parsing until we implement jobs.
    MPTask* task = nullptr;
    //XMLNode jobs(xmlFile, "Jobs");
    //for(auto& node : jobs)
    //  if(node.Name() == "Task") {
    //    task = new MPTask(problem, node);
    //    break;
    //  }

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
