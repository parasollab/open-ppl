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
typedef MPTraits<Cfg> PMPLTraits;

#elif (defined(PMPState))
#include "Cfg/State.h"
#include "Traits/StateTraits.h"
typedef StateTraits PMPLTraits;

#elif (defined(PMPCfgMultiRobot))
#include "Cfg/CfgMultiRobot.h"
#include "Traits/CfgTraits.h"
typedef MPTraits<CfgMultiRobot> PMPLTraits;

#elif (defined(PMPCfgSurface))
#include "Cfg/CfgSurface.h"
#include "Traits/SurfaceTraits.h"
typedef SurfaceTraits PMPLTraits;

#elif (defined(PMPReachDistCC))
#include "Cfg/Cfg_reach_cc.h"
typedef MPTraits<Cfg_reach_cc> PMPLTraits;

#elif (defined(PMPReachDistCCFixed))
#include "Cfg/Cfg_reach_cc_fixed.h"
typedef MPTraits<Cfg_reach_cc_fixed> PMPLTraits;

#elif (defined(PMPSSSurfaceMult))
#include "Cfg/SSSurfaceMult.h"
#include "Traits/SurfaceTraits.h"
typedef SSSurfaceMultTraits PMPLTraits;

#elif (defined(PMPReachableVolume))
#include "Cfg/CfgReachableVolume.h"
#include "Traits/ReachableVolumeTraits.h"
typedef CfgReachableVolumeTraits PMPLTraits;
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
    MPProblemType* problem = new MPProblemType(_argv[2]);
    problem->Solve();

    delete problem;

    return 0;
  }
  catch(const std::runtime_error& e) {
    cerr << endl << e.what() << endl;
    return 1;
  }
}

