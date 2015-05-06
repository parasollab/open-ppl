/* PMPL main function. Instantiates a problem from an input xml
 * filename. Then solves based upon the problem.
 */


#include "MPProblem/MPProblem.h"

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
#include "MPProblem/ClosedChainProblem.h"
#include "MPStrategies/ClosedChainStrategy.h"
#endif



#ifdef PMPCfg
#include "Cfg/Cfg.h"
#include "Traits/CfgTraits.h"
typedef Cfg PMPLCfgType;

#elif (defined(PMPCfgMultiRobot))
#include "Cfg/CfgMultiRobot.h"
#include "Traits/CfgTraits.h"
typedef CfgMultiRobot PMPLCfgType;

#elif (defined(PMPCfgSurface))
#include "Cfg/CfgSurface.h"
#include "Traits/SurfaceTraits.h"
typedef CfgSurface PMPLCfgType;

#elif (defined(PMPReachDistCC))
#include "Cfg/Cfg_reach_cc.h"
typedef Cfg_reach_cc PMPLCfgType;

#elif (defined(PMPReachDistCCFixed))
#include "Cfg/Cfg_reach_cc_fixed.h"
typedef Cfg_reach_cc_fixed PMPLCfgType;

#elif (defined(PMPSSSurfaceMult))
#include "Cfg/SSSurfaceMult.h"
#include "Traits/SurfaceTraits.h"
typedef SSSurfaceMult PMPLCfgType;

#elif (defined(PMPReachableVolume))
#include "Cfg/CfgReachableVolume.h"
#include "Traits/ReachableVolumeTraits.h"
typedef CfgReachableVolume PMPLCfgType;
#else
#error "Error, must define a RobotType for PMPL application"
#endif

using namespace std;

int main(int _argc, char** _argv) {

  if(_argc < 3 || !(string(_argv[1]) == "-f")) {
    cerr << "Error: Incorrect usage. Usage: -f options.xml" << endl;
    exit(1);
  }

  try {
    typedef MPTraits<PMPLCfgType> Traits;
    typedef Traits::MPProblemType MPProblemType;
    MPProblemType* problem = new MPProblemType(_argv[2]);
    problem->Solve();

    delete problem;
  }
  catch(const std::runtime_error& e) {
    cout << endl << e.what() << endl;
  }
  catch(...) {
    cout << "Unknown error." << endl;
  }

  return 0;
}


