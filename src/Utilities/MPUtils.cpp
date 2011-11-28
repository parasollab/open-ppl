#include "MPUtils.h"
#include "Clock_Class.h"
#include "MetricUtils.h"
#include "ValidityChecker.hpp"
#include "MPStrategy/MPStrategyMethod.h"
#include "MPProblem/MPProblem.h"
#include "MPRegion.h"

/*Basic utility for "growing" a RRT tree.  Assumed to be given a start node, and he goal node to grow towards.
Resulting node extended towards the goal is passed by reference and modified.  Returned boolean relays whether the
growth was succesful or not.

_mp -> Used to obtain information about the problem at hand
strVcmethod -> Used for VC calls
dm_label -> Used to extract DistanceMetricMethod pointer
start -> Location of Cfg on tree to grow from
dir -> Direction to grow towards
new_cfg -> New Cfg to be added to the tree; passed by reference
delta -> Maximum distance to grow
obsDist -> Distance away from object the new node neads to at least be


*/

bool RRTExpand( MPProblem* _mp, int _regionID, string _vc, string _dm, CfgType _start, CfgType _dir, CfgType& _newCfg, double _delta){
  //Setup...primarily for collision checks that occur later on
  MPRegion<CfgType,WeightType>*      region = _mp->GetMPRegion(_regionID);
  Stat_Class*                        regionStats = region->GetStatClass();
  Environment*                       env = region->GetRoadmap()->GetEnvironment();
  shared_ptr <DistanceMetricMethod>  dm = _mp->GetDistanceMetric()->GetDMMethod(_dm);
  ValidityChecker<CfgType>*          vc = _mp->GetValidityChecker();
  CDInfo                             cdInfo;
  string callee("RRTUtility::RRTExpand");

  vector<CfgType>::iterator startCIterator;
  vector<CfgType> cfgs;
  //CfgType cfg = _start;
  CfgType incr, tick = _start, previous = _start;
  bool collision = false;
  double positionRes    = env->GetPositionRes();
  double orientationRes = env->GetOrientationRes();
  int nTicks, ticker = 0;

  incr.FindIncrement(tick,_dir,&nTicks,positionRes,orientationRes);

  //Move out from start towards dir, bounded by number of ticks allowed at a given resolution.  Delta + obsDist are
  //given to the function, and are user defined.
  while(!collision && dm->Distance(env,_start,tick) <= _delta) {
    tick.Increment(incr); //Increment tick
    if(!(tick.InBoundingBox(env)) || !(vc->IsValid(vc->GetVCMethod(_vc), tick, env, *regionStats, cdInfo, true, &callee))){
      collision = true; //Found a collision; return previous tick, as it is collision-free
    }
    else{
      previous = tick;
    }
    ++ticker;
    if (ticker == nTicks){ //Have we reached the max amount of increments?
      break;
    }
  }
  if(previous != _start){ //Did we go anywhere?
    _newCfg = previous;//Last Cfg pushed back is the final tick allowed
    return true;     
  }
  //Didn't find a place to go :(
  else
    return false;
}

