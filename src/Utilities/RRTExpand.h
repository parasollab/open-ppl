#ifndef RRTEXPAND_H_
#define RRTEXPAND_H_

#include "MPProblem/Environment.h"

class StatClass;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// RRTExpand
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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

template<class MPTraits>
bool
RRTExpand(typename MPTraits::MPProblemType* _mp,
    string _vc, string _dm,
    const typename MPTraits::CfgType& _start,
    const typename MPTraits::CfgType& _dir,
    typename MPTraits::CfgType& _newCfg,
    double _delta, int& _weight, CDInfo& _cdInfo,
    double _posRes, double _oriRes){

  //Setup...primarily for collision checks that occur later on
  Environment* env = _mp->GetEnvironment();
  typename MPTraits::MPProblemType::DistanceMetricPointer dm = _mp->GetDistanceMetric(_dm);
  typename MPTraits::MPProblemType::ValidityCheckerPointer vc = _mp->GetValidityChecker(_vc);
  string callee("RRTUtility::RRTExpand");

  typename vector<typename MPTraits::CfgType>::iterator startCIterator;
  typename MPTraits::CfgType incr, tick = _start, previous = _start;
  bool collision = false;
  int nTicks, ticker = 0;

  incr.FindIncrement(tick,_dir,&nTicks, _posRes, _oriRes);
  _weight = nTicks;

  //Move out from start towards dir, bounded by number of ticks allowed at a
  //given resolution and the distance _delta: the maximum distance to grow
  while(!collision && dm->Distance(_start, tick) <= _delta && ticker <= nTicks) {
    previous = tick;
    tick += incr;
    if(!env->InBounds(tick) || !(vc->IsValid(tick, _cdInfo, callee)))
      collision = true; //return previous tick, as it is collision-free

    ++ticker;
  }
  if(previous != _start){
    _newCfg = previous;//Last Cfg pushed back is the final tick allowed
    return true;
  }
  else
    return false;
}

#endif
