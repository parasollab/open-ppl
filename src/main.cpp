#ifdef PMPCfg
#include "MPProblem/ConfigurationSpace/Cfg.h"
#include "Traits/CfgTraits.h"
typedef MPTraits<Cfg> PMPLTraits;

#elif (defined(PMPState))
#include "MPProblem/ConfigurationSpace/State.h"
#include "Traits/StateTraits.h"
typedef StateTraits PMPLTraits;

#else
#error "Error, must define a RobotType for PMPL application"
#endif

using namespace std;

int
main(int _argc, char** _argv) {

  try {
    if(_argc < 3 || string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    typedef PMPLTraits::MPProblemType MPProblemType;
    typedef PMPLTraits::PlanningLibraryType PlanningLibraryType;
    MPProblemType* problem = new MPProblemType(_argv[2]);
    PlanningLibraryType* lib = new PlanningLibraryType(_argv[2]);
    lib->SetMPProblem(problem);
    lib->Solve();

    delete problem;
    delete lib;

    return 0;
  }
  catch(const std::runtime_error& e) {
    cerr << endl << e.what() << endl;
    return 1;
  }
  catch(...) {
    cerr << "Unknown error." << endl;
    return 1;
  }
}

