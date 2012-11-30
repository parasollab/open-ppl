/* PMPL main function. Instantiates a problem from an input xml
 * filename. Then solves based upon the problem.
 */

#include "MPProblem/MPTraits.h"
#include "MPProblem/MPProblem.h"

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
#include "MPProblem/ClosedChainProblem.h"
#include "MPStrategy/ClosedChainStrategy.h"
#endif

#if (defined(PMPManifold))
#include "Cfg/ManifoldCfg.h"
typedef ManifoldCfg PMPLCfgType;
#elif (defined(PMPCfg2D))
#include "Cfg/Cfg_2D.h"
typedef Cfg_2D PMPLCfgType;
#elif (defined(PMPCfg2DWithRot))
#include "Cfg/Cfg_2D_withRot.h"
typedef Cfg_2D_withRot PMPLCfgType;
#elif (defined(PMPCfgSurface))
#include "Cfg/Cfg_surface.h"
typedef Cfg_surface PMPLCfgType;
#elif (defined(PMPReachDistCC))
#include "Cfg/Cfg_reach_cc.h"
typedef Cfg_reach_cc PMPLCfgType;
#elif (defined(PMPReachDistCCFixed))
#include "Cfg/Cfg_reach_cc_fixed.h"
typedef Cfg_reach_cc_fixed PMPLCfgType;
#else
#error "Error, must define a RobotType for PMPL application"
#endif

using namespace std;

#ifdef _PARALLEL
void stapl_main(int _argc, char *_argv[])
#else 
int main(int _argc, char** _argv)
#endif
{
  if(_argc < 3){ 
    cerr << "Error: Incorrect usage. Usage: -f options.xml" << endl;
    exit(1);
  }

  if(!(string(_argv[1]) == "-f")){
    cerr << "Error: Incorrect usage. Usage: -f options.xml" << endl;
    exit(1);
  }

  typedef MPTraits<PMPLCfgType> Traits;
  typedef typename Traits::MPProblemType MPProblemType;
  MPProblemType* problem = new MPProblemType(_argv[2]);
  problem->PrintOptions(cout);
  problem->Solve();

  return 0;
}


