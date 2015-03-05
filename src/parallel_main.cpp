/* PMPL main function. Instantiates a problem from an input xml
 * filename. Then solves based upon the problem.
 */

#include "MPProblem/MPProblem.h"
#include "Traits/ParallelCfgTraits.h"

#if (defined(PMPCfg))
#include "Cfg/Cfg.h"
typedef Cfg PMPLCfgType;
#elif (defined(PMPCfgSurface))
#include "Cfg/CfgSurface.h"
typedef CfgSurface PMPLCfgType;
#elif (defined(PMPReachDistCC))
#include "Cfg/Cfg_reach_cc.h"
typedef Cfg_reach_cc PMPLCfgType;
#elif (defined(PMPReachDistCCFixed))
#include "Cfg/Cfg_reach_cc_fixed.h"
typedef Cfg_reach_cc_fixed PMPLCfgType;
#elif (defined(PMPSSSurfaceMult))
#include "Cfg/SSSurfaceMult.h"
typedef SSSurfaceMult PMPLCfgType;
#else
#error "Error, must define a RobotType for PMPL application"
#endif

using namespace std;

stapl::exit_code stapl_main(int _argc, char* _argv[]) {

  if(_argc < 3 || !(string(_argv[1]) == "-f")){
    cerr << "Error: Incorrect usage. Usage: -f options.xml" << endl;
    exit(1);
  }

  typedef MPTraits<PMPLCfgType> Traits;
  typedef Traits::MPProblemType MPProblemType;
  MPProblemType* problem = new MPProblemType(_argv[2]);
  problem->Print(cout);
  problem->Solve();

  delete problem;

  return EXIT_SUCCESS;
}


