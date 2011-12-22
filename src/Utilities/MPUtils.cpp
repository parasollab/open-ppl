#include "MPUtils.h"
#include "MetricUtils.h"
#include "CollisionDetectionValidity.hpp"
#include "ValidityChecker.hpp"
#include "Sampler.h"
#include "SamplerMethod.h"
#include "LocalPlanners.h"
#include "LocalPlannerMethod.h"
#include "MPStrategy.h"
#include "MPProblem.h"
#include "MPRegion.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Random Number Generation
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

double DRand() {return drand48();}
long LRand() {return lrand48();}
long MRand() {return mrand48();}

// normally(gaussian) distributed random number generator.
double GRand(bool _reset) {
  double v1, v2, rsq;
  static int iset = 0;
  if(_reset)  { //reset iset to 0 and return
    iset = 0;
    return 0.0;
  }
  static double gset;
  if(iset == 0) {
    do {
      v1 = 2*DRand() - 1.0;
      v2 = 2*DRand() - 1.0;
      rsq = v1*v1 + v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    double fac = sqrt(-2.0*log(rsq)/rsq);
    gset = v1*fac;
    iset = 1;
    return v2*fac;
  } else {
    iset = 0;
    return gset;
  }
}

long SRand(long _seedVal){
  static long oldSeed = _seedVal;
  if(oldSeed != _seedVal) {
    oldSeed = _seedVal;
    return SRand("NONE", 0, _seedVal, true);
  } 
  else
    return SRand("NONE", 0, _seedVal);
}

//the real seed is decided by: baseSeed, methodName, nextNodeIndex
long SRand(string _methodName, int _nextNodeIndex, long _base, bool _reset) {
  static long baseSeed = _base;
  if(_reset)
    baseSeed = _base;
  if(_methodName != "NONE") {
    long methodID = 0;
    for(size_t i=0; i<_methodName.length(); i++){
      int tmp = _methodName[i];
      methodID += tmp*(i+1)*(i+2);
    }
    srand48(long (baseSeed * (_nextNodeIndex+1) + methodID));
  } 
  else {
    srand48(baseSeed);
  }
  return baseSeed;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Simple Distance Utilities
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Calculate the minimum DIRECTED angular distance between two angles
// normalized to 1.0
double DirectedAngularDistance(double _a,double _b) {
  // normalize both a and b to [0, 1)
  _a = _a - floor(_a);
  _b = _b - floor(_b);

  // shorten the distance between a and b by shifting a or b by 1.
  // have to do ROUND-UP INTEGER comparision to avoid subtle numerical errors.
  int intA = (int)rint(_a*1000000);
  int intB = (int)rint(_b*1000000);

  if( intB - intA  > 500000 ) 
    ++_a;
  else if ( intA - intB > 500000 )
    ++_b;
  return _b-_a;
}

double GaussianDistribution(double _mean, double _stdev) {
  double x1, x2, w, r;
  do {
    r = DRand();
    x1 = 2. * r - 1.;
    r = DRand();
    x2 = 2. * r - 1.;
    w = x1 * x1 + x2 * x2;
  } while(w >= 1. || w < 1E-30);
  w = sqrt((-2. * log(w)) / w);
  x1 *= w;
  return x1 * _stdev + _mean;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// MPProblem Access Helpers
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<SamplerMethod<CfgType> > GetSamplingMethod(MPProblem* _mp, string _s){
  return _mp->GetMPStrategy()->GetSampler()->GetSamplingMethod(_s);
};

boost::shared_ptr<LocalPlannerMethod<CfgType, WeightType> > GetLPMethod(MPProblem* _mp, string _s){
  return _mp->GetMPStrategy()->GetLocalPlanners()->GetLocalPlannerMethod(_s);
};

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

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//
// Medial Axis Utility
//
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//***********************************//
// Main Push To Medial Axis Function //
//***********************************//
bool PushToMedialAxis(MPProblem* _mp, Environment* _env, CfgType& _cfg, Stat_Class& _stats, string _vc, string _dm, 
    bool _cExact, int _clearance, bool _pExact, int _penetration, bool _useBBX, double _eps, int _hLen, bool _debug) {
  // Initialization
  string call("MedialAxisUtility::PushToMedialAxis()");
  if (_debug) cout << endl << call << endl << "CfgType: " << _cfg;
  ValidityChecker<CfgType>* vc = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);
  bool inside, inCollision, found, pushed=true;
  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;

  // If invalid, push to the outside of the obstacle
  inside = vcm->isInsideObstacle(_cfg, _env, tmpInfo);
  inCollision = !(vc->IsValid(vcm, _cfg, _env, _stats,tmpInfo,true,&call));
  if (_debug) cout << " Inside/In-Collision: " << inside << "/" << inCollision << endl;
  if (inside || inCollision)
    pushed = PushFromInsideObstacle(_mp,_cfg,_env,_stats,_vc,_dm,_pExact,_penetration,_debug);
  if ( !pushed ) 
    return false;

  // Cfg is free, find medial axis
  found = PushCfgToMedialAxis(_mp,_cfg,_env,_stats,_vc,_dm,_cExact,_clearance,_useBBX,_eps,_hLen,_debug);
  if ( !found )  { 
    if (_debug) cout << "Not found!! ERR in pushtomedialaxis" << endl; 
    return false; 
  }

  if (_debug) cout << "FINAL CfgType: " << _cfg << endl << call << "::END" << endl << endl;
  return true;
} // END pushToMedialAxis

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
bool PushFromInsideObstacle(MPProblem* _mp, CfgType& _cfg, Environment* _env, Stat_Class& _stats,
    string _vc, string _dm, bool _pExact, int _penetration, bool _debug) {	
  // Initialization
  string call("MedialAxisUtility::PushFromInsideObstacle");
  if (_debug) cout << call << endl << " CfgType: " << _cfg << endl;
  shared_ptr<DistanceMetricMethod>  dm  = _mp->GetDistanceMetric()->GetDMMethod(_dm);
  ValidityChecker<CfgType>*             vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);

  // Variables
  CDInfo   tmpInfo;
  Vector3D transDir, dif;
  vector<double> gDif;
  bool     valid, inBBX;
  CfgType      endCfg=_cfg, tmpCfg=_cfg, heldCfg=_cfg, transCfg=_cfg;
  double   stepSize=1.0, res=_env->GetPositionRes(), factor; // TODO: ori_res=env->GetOrientationRes()
  bool     initValidity = vc->IsValid(vcm,_cfg,_env,_stats,tmpInfo,true,&call);
  bool     tmpValidity = false, prevValidity = false;

  // If in collision (using the exact case), must use approx
  if ( initValidity == false )
    _pExact = false;

  // Get Collision info, if not computable, exit
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
  valid = CalculateCollisionInfo(_mp,_cfg,_env,_stats,tmpInfo,_vc,_dm,_pExact,0,_penetration,true);
  if (!valid)	return false;

  // Determine direction to move
  transDir = tmpInfo.object_point - tmpInfo.robot_point;
  for (int i=0; i<transCfg.DOF(); i++) {
    if ( i < transCfg.PosDOF() ) 
      transCfg.SetSingleParam(i, transDir[i]);
    else                          
      transCfg.SetSingleParam(i, 0.0);
  }
  dif[0] = transCfg.GetSingleParam(0);
  dif[1] = transCfg.GetSingleParam(1);
  dif[2] = transCfg.GetSingleParam(2);
  factor = res/sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
  transCfg.multiply(transCfg,factor);
  if (_debug) cout << "TRANS CfgType: " << transCfg << endl;

  // Check if valid and outside obstacle
  while ( !tmpValidity ) {
    heldCfg = tmpCfg;
    tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);
    tmpValidity = vc->IsValid(vcm,tmpCfg,_env,_stats,tmpInfo,true,&call);
    tmpValidity = tmpValidity && !vcm->isInsideObstacle(tmpCfg,_env,tmpInfo);
    if ( tmpValidity ) {
      tmpValidity = tmpValidity && prevValidity;
      if ( !prevValidity )
        prevValidity = true;
    }
    inBBX = tmpCfg.InBoundingBox(_env);
    if ( !inBBX ) { 
      if (_debug) cout << "Fell out of BBX, error out... " << endl;
      return false;
    }
    stepSize += 1.0;
  }
  _cfg = tmpCfg;
  if (_debug) cout << "FINAL CfgType: " << _cfg << " steps: " << stepSize-1.0 << endl << call << "::END " << endl;
  return true;
}

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
bool PushCfgToMedialAxis(MPProblem* _mp, CfgType& _cfg, Environment* _env, Stat_Class& _stats,
    string _vc, string _dm, bool _cExact, int _clearance, bool _useBBX, double _eps, int _hLen, bool _debug) {
  // Initialization
  string call("MedialAxisUtility::PushCfgToMedialAxis");
  if (_debug) cout << call << endl << "CfgType: " << _cfg << " eps: " << _eps << endl;
  shared_ptr<DistanceMetricMethod>  dm  = _mp->GetDistanceMetric()->GetDMMethod(_dm);
  ValidityChecker<CfgType>*             vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);

  // Variables
  Vector3D transDir, dif;
  vector<double> gDif;
  CDInfo   tmpInfo;
  CfgType  transCfg, tmpCfg, heldCfg;
  double   stepSize=0.0, cbStepSize=0.0, factor=0.0, res=_env->GetPositionRes();// TODO: ori_res=env->GetOrientationRes();
  bool     inBBX=true, goodTmp=true, valid, inside=vcm->isInsideObstacle(_cfg,_env,tmpInfo);
  if ( inside ) 
    return false;

  // tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
  tmpCfg = _cfg;

  // Determine direction to move and clearance
  valid = CalculateCollisionInfo(_mp,_cfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
  if ( !valid ) 
    return false;

  transDir = tmpInfo.robot_point - tmpInfo.object_point;
  for (int i=0; i<transCfg.DOF(); i++) {
    if ( i < transCfg.PosDOF() ) 
      transCfg.SetSingleParam(i, transDir[i]);
    else                          
      transCfg.SetSingleParam(i, 0.0);
  }
  dif[0] = transCfg.GetSingleParam(0);
  dif[1] = transCfg.GetSingleParam(1);
  dif[2] = transCfg.GetSingleParam(2);
  factor = res/sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
  transCfg.multiply(transCfg,factor);
  if (_debug) cout << "TRANS CfgType: " << transCfg << endl;

  // Initialize temp Info
  int segLen=_hLen, posCnt=0, negCnt=0, zroCnt=0, stepsTaken, tcc=0;
  bool peakFound = false, broke=false, checkBack=false;
  //vector<double> segDists(segLen,0), seg_deltas(segLen-1,0);
  deque<double> segDists;
  deque<CfgType> segCfgs;
  double maxDist=0.0;

  // Determine gap for medial axis
  while ( !peakFound ) {

    // Increment
		if ( checkBack ) tmpCfg.multiply(transCfg,cbStepSize);
    else             tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);
    inside = vcm->isInsideObstacle(tmpCfg,_env,tmpInfo);
    inBBX = tmpCfg.InBoundingBox(_env);

    bool tmpVal = (inside || !inBBX);
    VDAddTempCfg(tmpCfg, tmpVal);
    tcc++;

    // If inside obstacle or out of the bbx, step back
    if ( inside || !inBBX) {
      if ( !inBBX && !_useBBX ) 
        return false;
      if ( segCfgs.size() > 0 ) 
        break;
      else                       
        return false;
    } 
    else {
      // If tmp is valid, move gap on to next step
      if ( vc->IsValid(vcm,tmpCfg,_env,_stats,tmpInfo,true,&call) ) {
        if (_debug) cout << "TMP CfgType: " << tmpCfg;
        goodTmp = CalculateCollisionInfo(_mp,tmpCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
        if ( !goodTmp && broke ) 
          return false; // If can't compute twice in a row, error out.
        broke = !goodTmp;
        if ( goodTmp ) {
          if (_debug) cout << "  clearance: " << tmpInfo.min_dist;
          ++stepsTaken;
          if ( int(segDists.size()) > segLen ) {
            segDists.erase(segDists.begin());
            segCfgs.erase(segCfgs.begin());
          }
          if ( checkBack ) {
            segDists.push_front(tmpInfo.min_dist);
            segCfgs.push_front(tmpCfg);
          } else {
            segDists.push_back(tmpInfo.min_dist);
            segCfgs.push_back(tmpCfg);
            checkBack=false;
          }
        } 
        else {
          if (_debug) cout << "BROKE!" << endl;
          return false;
        }
        stepSize += 1.0;

        maxDist = segDists[0]; posCnt=0; negCnt=0; zroCnt=0;
        for ( int i=0; i<int(segDists.size())-1; i++ ) {
          double tmp = segDists[i+1] - segDists[i];

          if      ( tmp > 0.0 ) ++posCnt;
          else if ( tmp < 0.0 ) ++negCnt;
          else                  ++zroCnt;
          maxDist = (maxDist>segDists[i+1])?maxDist:segDists[i+1];
        }
        if (_debug) cout << "  Pos/Zero/Neg counts: " << posCnt << "/" << zroCnt << "/" << negCnt << endl;
        if ( (negCnt > 0) && (negCnt >= posCnt) ) {
          if (posCnt < 1 && zroCnt < 1) { // Only see negative, check back
            --cbStepSize;
            checkBack = true;
          } else {
            peakFound = true;
            if (_debug) {
              cout << "Found peak!  Dists: ";
              for (int i=0; i<int(segDists.size()); i++)
                cout << segDists[i] << " ";
              cout << endl;
            }
          }
        } else if ( (zroCnt > 0) && (zroCnt >= posCnt) ) {
          if (posCnt < 1 && zroCnt < 1) { // Only see zeros, check back
            --cbStepSize;
            checkBack = true;
          } else {
            peakFound = true;
            if (_debug) {
              cout << "Found peak!  Dists: ";
              for (int i=0; i<int(segDists.size()); i++)
                cout << segDists[i] << " ";
              cout << endl;
            }
          }
        } else {
          checkBack = false;
        }

      } 
      else { // Else reduce step size or fall out of the loop
        return false;
      }
    }
  }

  for (int i=0; i<tcc; i++)
    VDClearLastTemp();

  // Variables for modified binary search
  CfgType startCfg, midSCfg, midMCfg, midECfg, endingCfg;
  Cfg* difCfg = _cfg.CreateNewCfg();
  double gapDist, prevGap;
  int attempts=0, maxAttempts=20, badPeaks=0, maxBadPeaks=10, peak;
  bool peaked=false;
  vector<double> dists(5,0), deltas(4,0);

  // Setup start, middle and end CfgTypes  
  startCfg = segCfgs[0];
  endingCfg = segCfgs[segCfgs.size()-1];
  midMCfg.add(startCfg, endingCfg);
  midMCfg.divide(midMCfg,2);
  CalculateCollisionInfo(_mp,startCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
  dists[0] = tmpInfo.min_dist;
  CalculateCollisionInfo(_mp,midMCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
  dists[2] = tmpInfo.min_dist;
  CalculateCollisionInfo(_mp,endingCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
  dists[4] = tmpInfo.min_dist;

  if (_debug) cout << "start/mid/end: " << endl << startCfg << endl << midMCfg << endl << endingCfg << endl;

  midSCfg.add(startCfg, midMCfg);
  midSCfg.divide(midSCfg,2);
  midECfg.add(midMCfg, endingCfg);
  midECfg.divide(midECfg,2);

  if (_debug) {
    cout << "dists: ";
    for (int i=0; i<int(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }

  // TODO: Update to DIST, not GAP
  // Get collision info and gap distance
  difCfg->subtract(startCfg,endingCfg);
  gDif.clear();
  for(int i=0; i<startCfg.DOF(); ++i) {
    if ( i < startCfg.PosDOF() ) 
      gapDist += (startCfg.GetSingleParam(i) - endingCfg.GetSingleParam(i))*
        (startCfg.GetSingleParam(i) - endingCfg.GetSingleParam(i));
    else                          
      gapDist += (DirectedAngularDistance(startCfg.GetSingleParam(i), endingCfg.GetSingleParam(i)))*
        (DirectedAngularDistance(startCfg.GetSingleParam(i), endingCfg.GetSingleParam(i)));
  }
  gapDist = sqrt(gapDist);
  prevGap = gapDist;

  do { // Modified Binomial search to find peaks
    ostringstream strs;
    strs << "Peak Clearance: " << dists[2];
    VDComment(strs.str());
    VDAddTempCfg(midMCfg, true);
    VDClearLastTemp();
    VDClearComments();

    attempts++;
    midSCfg.add(startCfg, midMCfg); 
    midSCfg.divide(midSCfg,2);
    midECfg.add(midMCfg, endingCfg); 
    midECfg.divide(midECfg,2);

    CalculateCollisionInfo(_mp,midSCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
    dists[1] = tmpInfo.min_dist;
    CalculateCollisionInfo(_mp,midECfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX);
    dists[3] = tmpInfo.min_dist;

    // Compute Deltas and Max Distance
    deltas.clear();
    maxDist = (dists[0]);
    if (_debug) cout << "Deltas: ";
    for ( int i=0; i<int(dists.size())-1; i++ ) {
      double tmp = dists[i+1] - dists[i];
      if (_debug) cout << tmp << " ";
      deltas.push_back(tmp);
      maxDist = (maxDist>dists[i+1])?maxDist:dists[i+1];
    } 
    if (_debug) cout << endl;

    // Determine Peak
    if ( deltas[0] > 0 && deltas[1] <= 0 && dists[1] == maxDist) {
      peak = 1;
      endingCfg = midMCfg;
      midMCfg = midSCfg;
      dists[4] = dists[2];
      dists[2] = dists[1];
    } 
    else if ( deltas[1] > 0 && deltas[2] <= 0 && dists[2] == maxDist) {
      peak = 2;
      startCfg = midSCfg;
      endingCfg = midECfg;
      dists[0] = dists[1];
      dists[4] = dists[3];
    } 
    else if ( deltas[2] > 0 && deltas[3] <= 0 && dists[3] == maxDist) {
      peak = 3;
      startCfg = midMCfg;
      midMCfg = midECfg;
      dists[0] = dists[2];
      dists[2] = dists[3];
    } 
    else {
      if ( deltas[0] > 0 && deltas[1] > 0 && deltas[2] > 0 && deltas[3] > 0) {
        peak = 3;
        startCfg = midMCfg;
        midMCfg = midECfg;
        dists[0] = dists[2];
        dists[2] = dists[3];
      } 
      else if ( deltas[0] < 0 && deltas[1] < 0 && deltas[2] < 0 && deltas[3] < 0) {
        peak = 1;
        endingCfg = midMCfg;
        midMCfg = midSCfg;
        dists[4] = dists[2];
        dists[2] = dists[1];
      } 
      else {
        // No peak found, recalculate old, mid and new clearance
        badPeaks++;
        peak = -1;
        if ( CalculateCollisionInfo(_mp,startCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX) 
             && dists[0] >= tmpInfo.min_dist )
          dists[0] = tmpInfo.min_dist;
        if ( CalculateCollisionInfo(_mp,midMCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX) 
             && dists[2] >= tmpInfo.min_dist )
          dists[2] = tmpInfo.min_dist;
        if ( CalculateCollisionInfo(_mp,endingCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX) 
             && dists[4] >= tmpInfo.min_dist )
          dists[4] = tmpInfo.min_dist;
      }
    }
    if (_debug) cout << " peak: " << peak << "  cfg: " << midMCfg;

    // TODO: Update to DIST, not GAP
    // Check if minimum gap distance has been acheived
    difCfg->subtract(startCfg,endingCfg);
    gDif.clear(); 
    gapDist = 0.0;
    for(int i=0; i<startCfg.DOF(); ++i) {
      if ( i < startCfg.PosDOF() ) gapDist += (startCfg.GetSingleParam(i) - endingCfg.GetSingleParam(i))*
        (startCfg.GetSingleParam(i) - endingCfg.GetSingleParam(i));
      else                          
        gapDist += (DirectedAngularDistance(startCfg.GetSingleParam(i), endingCfg.GetSingleParam(i)))*
          (DirectedAngularDistance(startCfg.GetSingleParam(i), endingCfg.GetSingleParam(i)));
    }

    gapDist = sqrt(gapDist);
    if (_debug) cout << " gap: " << gapDist << endl;
    if ( _eps >= gapDist )
      peaked = true;

  } while ( !peaked && attempts < maxAttempts && badPeaks < maxBadPeaks );

  _cfg = midMCfg;
  if (_debug) cout << "FINAL CfgType: " << _cfg << " steps: " << stepSize << endl;
  return true;
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
bool CalculateCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, Stat_Class& _stats, CDInfo& _cdInfo, 
    string _vc, string _dm, bool _exact, int _clearance, int _penetration, bool _useBBX) {
  if ( _exact ) 
    return GetExactCollisionInfo(_mp,_cfg,_env,_stats,_cdInfo,_vc,_useBBX);
  else         
    return GetApproxCollisionInfo(_mp,_cfg,_env,_stats,_cdInfo,_vc,_dm,_clearance,_penetration,_useBBX);
}

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
bool GetExactCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, Stat_Class& _stats,
    CDInfo& _cdInfo, string _vc, bool _useBBX) {
  // Setup Validity Checker
  string call("MedialAxisUtility::getExactCollisionInfo");
  ValidityChecker<CfgType>*         vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);
  _cdInfo.ResetVars();
  _cdInfo.ret_all_info = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  if ( !_cfg.InBoundingBox(_env) || !(vc->IsValid(vcm,_cfg,_env,_stats,_cdInfo,true,&call)) ) 
    return false;

  // If not using the bbx, done
  if ( !_useBBX )
    return true;

  // CfgType is now know as good, get BBX and ROBOT info
  boost::shared_ptr<BoundingBox> bbx = _env->GetBoundingBox();
  boost::shared_ptr<MultiBody> robot = _env->GetMultiBody(_env->GetRobotIndex());
  std::pair<double,double> bbxRange;

  // TODO: Update to use DIST, not literal
  // Find closest point between robot and bbx, set if less than min dist from obstacles
  for(int m=0; m < robot->GetFreeBodyCount(); m++) {
    GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
    for(size_t j = 0 ; j < poly.vertexList.size() ; j++){
      for (int k=0; k<_cfg.PosDOF(); k++) { // For all positional DOFs
        bbxRange = bbx->GetRange(k);
        if ( (poly.vertexList[j][k]-bbxRange.first) < _cdInfo.min_dist ) {
          for (int l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.robot_point[l]  = poly.vertexList[j][l];
            _cdInfo.object_point[l] = poly.vertexList[j][l];
          }
          _cdInfo.object_point[k] = bbxRange.first; // Lower Bound
          _cdInfo.min_dist = poly.vertexList[j][k]-bbxRange.first;
          _cdInfo.nearest_obst_index = -(k+1); // Different bbx face
        }
        if ( (bbxRange.second-poly.vertexList[j][k]) < _cdInfo.min_dist ) {
          for (int l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.robot_point[l]  = poly.vertexList[j][l];
            _cdInfo.object_point[l] = poly.vertexList[j][l];
          }
          _cdInfo.object_point[k] = bbxRange.second; // Upper Bound
          _cdInfo.min_dist = bbxRange.second-poly.vertexList[j][k];
          _cdInfo.nearest_obst_index = -(k+1);
        }
      }
    }
  }
  return (_cdInfo.min_dist>=0)?true:false;
} // END getExactCollisionInfo

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
bool GetApproxCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, Stat_Class& _stats,
    CDInfo& _cdInfo, string _vc, string _dm, int _clearance, int _penetration, bool _useBBX) {

  // Initialization
  string call = "MedialAxisUtility::getApproxCollisionInfo";
  shared_ptr<DistanceMetricMethod>  dm  = _mp->GetDistanceMetric()->GetDMMethod(_dm);
  ValidityChecker<CfgType>*         vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);

  // If in BBX, check validity to get _cdInfo, return false if not valid
  if ( !_cfg.InBoundingBox(_env) ) 
    return false;
  _cdInfo.ResetVars();
  _cdInfo.ret_all_info = true;	
  bool initInside   = vcm->isInsideObstacle(_cfg,_env,_cdInfo);                  // Initially Inside Obst
  bool initValidity = vc->IsValid(vcm,_cfg,_env,_stats,_cdInfo,true,&call);       // Initial Validity
  bool initInBBX    = _cfg.InBoundingBox(_env);
  initValidity = initValidity && !initInside;
  if ( _useBBX ) 
    initValidity = (initValidity && initInBBX);

  // Set Robot point in positional space // TODO: Higher DOF
  for (int i=0; i<_cfg.PosDOF(); i++)
    _cdInfo.robot_point[i] = _cfg.GetSingleParam(i);

  // Setup Major Variables and Constants:
  CDInfo tmpInfo;
  Vector3D dif;
  int numRays;
  double res=_env->GetPositionRes();// TODO: ori_res=env->GetOrientationRes();

  // Generate 'numRays' random directions at a distance 'dist' away from 'cfg'
  if ( initValidity ) 
    numRays = (initInside)?_penetration:_clearance;
  else                 
    numRays = _penetration;    

  vector<Cfg*> directions, candIn, candOut;
  vector<Cfg*> tick;
  vector<pair<Cfg*,double> > incr;

  // Setup translation rays
  for(int i=0; i<numRays; i++) {
    Cfg* tmp1 = _cfg.CreateNewCfg();
    tick.push_back(tmp1);
    Cfg* tmp2 = _cfg.CreateNewCfg();
    tmp2->GetRandomRayPos(res, _env);
    incr.push_back(make_pair(tmp2, tmp2->OrientationMagnitude() +
          tmp2->PositionMagnitude()));
  }

  // Setup to step out along each direction:
  bool stateChangedFlag = false, currValidity, currInside, currInBBX;
  size_t lastLapIndex = -1, endIndex = -1;
  int iterations = 0, maxIterations=1000, maxNumSteps = 25; // Arbitrary //TODO: Smarter number

  // Shoot out each ray to determine the candidates
  while ( !stateChangedFlag && iterations < maxIterations ) {
    iterations++;
    // For Each Ray
    for ( size_t i=0; i<incr.size(); ++i ) {
      tick[i]->Increment(*incr[i].first);
      currInside   = vcm->isInsideObstacle(*tick[i],_env,_cdInfo);
      currValidity = vc->IsValid(vcm,*tick[i],_env,_stats,tmpInfo,true,&call);
      currInBBX    = tick[i]->InBoundingBox(_env);
      currValidity = currValidity && !currInside;
      if ( _useBBX ) 
        currValidity = (currValidity && currInBBX);
      if ( currValidity != initValidity ) { // If Validity State has changed
        if ( lastLapIndex != i ) {
          Cfg* candO = tick[i]->CreateNewCfg();
          Cfg* candI = tick[i]->CreateNewCfg();
          candI->subtract(*candI,*incr[i].first);
          candOut.push_back(candO);
          candIn.push_back(candI);
        } 
        else
          stateChangedFlag = true; // lastLapIndex == i, made full pass
        if ( lastLapIndex == endIndex )  // Set lastLapIndex to first ray changing state
          lastLapIndex = i;
      }
      if ( stateChangedFlag ) break; // Once validity changes, exit loop
    } // End for
  } // End while

  if ( candIn.size() == 0 ) 
    return false;

  // Refine each candidate to find the best one
  _cdInfo.min_dist = MAX_DBL;
  bool midValidity;

  Cfg* innerCfg = candIn[0]->CreateNewCfg();
  Cfg* outerCfg = candOut[0]->CreateNewCfg();
  Cfg* midCfg = _cfg.CreateNewCfg();
  Cfg* difCfg  = _cfg.CreateNewCfg();
  double tmpDist;

  for ( size_t i=0; i<candOut.size(); i++ ) {
    innerCfg = candIn[i]->CreateNewCfg();
    outerCfg = candOut[i]->CreateNewCfg();
    midCfg   = _cfg.CreateNewCfg();
    int j=0;

    do {
      // Calculate new midpoint
      midCfg->add(*innerCfg,*outerCfg);
      midCfg->divide(*midCfg,2);
      midValidity = vc->IsValid(vcm,*midCfg,_env,_stats,tmpInfo,true,&call);

      if ( !(midCfg->InBoundingBox(_env)) || (midValidity != initValidity) ) { // If Outside BBX or Validity has changed
        if ( initValidity ) 
          *outerCfg = *midCfg;
        else                 
          *innerCfg = *midCfg;
      } else {
        if ( initValidity ) 
          *innerCfg = *midCfg;
        else                 
          *outerCfg = *midCfg;
      }

      // TODO: Update to use DIST, not literal
      // Compute Real Dist
      difCfg->subtract(*innerCfg,*outerCfg);
      tmpDist = 0.0;
      for (int j=0; j<_cfg.PosDOF(); j++)
        tmpDist += difCfg->GetSingleParam(j)*difCfg->GetSingleParam(j);
      tmpDist = sqrt(tmpDist);
    } while ((tmpDist > res/100.0) && (++j <= maxNumSteps));

    *midCfg = (initValidity)?*innerCfg:*outerCfg;
    difCfg->subtract(*midCfg,_cfg);
    tmpDist = 0.0;
    for (int j=0; j<_cfg.PosDOF(); j++)
      tmpDist += difCfg->GetSingleParam(j)*difCfg->GetSingleParam(j);
    tmpDist = sqrt(tmpDist);

    // Determine if new clearance is better than existing solution
    if ( _cdInfo.min_dist > tmpDist ) {
      if ( midCfg->InBoundingBox(_env) ) {
        _cdInfo.min_dist = tmpDist;
        for (int j=0; j<midCfg->PosDOF();j++)
          _cdInfo.object_point[j] = midCfg->GetSingleParam(j);
      } 
      else 
        _cdInfo.min_dist = 0;
    }
  }
  return (_cdInfo.min_dist == 0)?false:true;
} // END getApproxCollisionInfo

