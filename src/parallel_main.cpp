/* PMPL main function. Instantiates a problem from an input xml
 * filename. Then solves based upon the problem.
 */

#include "MPProblem/MPProblem.h"
#include "Traits/ParallelCfgTraits.h"

#if (defined(PMPCfg))
#include "MPProblem/ConfigurationSpace/Cfg.h"
typedef Cfg PMPLCfgType;
#else
#error "Error, must define a RobotType for PMPL application"
#endif

using namespace std;

stapl::exit_code
stapl_main(int _argc, char* _argv[]) {

  try {
    if(_argc < 3 || string(_argv[1]) != "-f")
      throw ParseException(WHERE, "Incorrect usage. Usage: -f options.xml");

    typedef MPTraits<PMPLCfgType> Traits;
    typedef Traits::MPProblemType MPProblemType;
    MPProblemType* problem = new MPProblemType(_argv[2]);
    stapl::rmi_fence();
    problem->Solve();

    delete problem;

    return EXIT_SUCCESS;
  }
  catch(const std::runtime_error& e) {
    cerr << endl << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch(...) {
    cerr << "Unknown error." << endl;
    return EXIT_FAILURE;
  }

}


