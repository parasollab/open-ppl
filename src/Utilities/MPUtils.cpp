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

boost::shared_ptr<LocalPlannerMethod<CfgType, WeightType> > GetLPMethod(MPProblem* _mp, string _s){
  return _mp->GetMPStrategy()->GetLocalPlanners()->GetMethod(_s);
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

bool RRTExpand( MPProblem* _mp, int _regionID, string _vc, string _dm, CfgType _start, CfgType _dir, CfgType& _newCfg, double _delta, CDInfo& _cdInfo){
  //Setup...primarily for collision checks that occur later on
  MPRegion<CfgType,WeightType>*      region = _mp->GetMPRegion(_regionID);
  StatClass*                        regionStats = region->GetStatClass();
  Environment*                       env = region->GetRoadmap()->GetEnvironment();
  shared_ptr <DistanceMetricMethod>  dm = _mp->GetDistanceMetric()->GetMethod(_dm);
  ValidityChecker<CfgType>*          vc = _mp->GetValidityChecker();
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
    if(!(tick.InBoundingBox(env)) || !(vc->IsValid(vc->GetVCMethod(_vc), tick, env, *regionStats, _cdInfo, true, &callee))){
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
bool PushToMedialAxis(MPProblem* _mp, Environment* _env, shared_ptr<BoundingBox> _bb, CfgType& _cfg, 
    StatClass& _stats, string _vc, string _dm, bool _cExact, int _clearance, bool _pExact, int _penetration, 
		bool _useBBX, double _eps, int _hLen, bool _debug, bool _positional) {

  // Initialization
  string call("MedialAxisUtility::PushToMedialAxis()");
  if (_debug) cout << endl << call << endl << "Being Pushed: " << _cfg;
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
    pushed = PushFromInsideObstacle(_mp,_cfg,_env,_bb,_stats,_vc,_dm,_pExact,_penetration,_debug,_positional);
  if ( !pushed ) 
    return false;

  // Cfg is free, find medial axis
  found = PushCfgToMedialAxis(_mp,_cfg,_env,_bb,_stats,_vc,_dm,_cExact,_clearance,_useBBX,_eps,_hLen,_debug,_positional);
  if ( !found )  { 
    if (_debug) cout << "Not found!! ERR in pushtomedialaxis" << endl; 
    return false; 
  }

  if (_debug) cout << "FINAL CfgType: " << _cfg << endl << call << "::END" << endl << endl;
  return true;
} // END pushToMedialAxis


bool PushToMedialAxis(MPProblem* _mp, Environment* _env, CfgType& _cfg, StatClass& _stats, string _vc, 
    string _dm, bool _cExact, int _clearance, bool _pExact, int _penetration, bool _useBBX, double _eps, 
    int _hLen, bool _debug, bool _positional) {
  return PushToMedialAxis( _mp,  _env, _env->GetBoundingBox(), _cfg, _stats, _vc, _dm, 
    _cExact, _clearance, _pExact, _penetration, _useBBX, _eps, _hLen, _debug, _positional);
}

//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//
bool PushFromInsideObstacle(MPProblem* _mp, CfgType& _cfg, Environment* _env, shared_ptr<BoundingBox> _bb, 
    StatClass& _stats,string _vc, string _dm, bool _pExact, int _penetration, bool _debug, bool _positional) {	
  // Initialization
  string call("MedialAxisUtility::PushFromInsideObstacle");
  if (_debug) cout << call << endl << " CfgType: " << _cfg << endl;
  shared_ptr<DistanceMetricMethod>  dm  = _mp->GetDistanceMetric()->GetMethod(_dm);
  ValidityChecker<CfgType>*         vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);

  // Variables
  CDInfo   tmpInfo;
  Vector3D transDir, dif;
  bool     inBBX;
  CfgType  endCfg=_cfg, tmpCfg=_cfg, heldCfg=_cfg, transCfg=_cfg;
  double   stepSize=1.0, posRes=_env->GetPositionRes(), oriRes=_env->GetOrientationRes(), factor;
  bool     initValidity=vc->IsValid(vcm,_cfg,_env,_stats,tmpInfo,true,&call);
  bool     tmpValidity=false, prevValidity=false;
  int      nTicks;

  // If in collision (using the exact case), must use approx
  if ( initValidity == false )
    _pExact = false;

  // Determine direction to move
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;

  if ( CalculateCollisionInfo(_mp,_cfg,transCfg,_env,_stats,tmpInfo,_vc,_dm,_pExact,0,_penetration,true,_positional) ) {
    if ( _pExact ) {
      Vector3D transDir = tmpInfo.object_point - tmpInfo.robot_point;
      for (int i=0; i<transCfg.DOF(); i++) {
        if ( i < transCfg.PosDOF() ) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i],2);
        } else transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg,posRes/sqrt(factor));
    } else {
      if (_debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg,transCfg,&nTicks,posRes,oriRes);
      for (int i=transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if ( transCfg.GetSingleParam(i) > 0.5 )
          transCfg.SetSingleParam(i,transCfg.GetSingleParam(i)-1.0, false);
      }
    }
  if (_debug) cout << "TRANS CFG: " << transCfg << endl;
  } else return false;

  // Check if valid and outside obstacle
  while ( !tmpValidity ) {
    tmpInfo.ResetVars();
    tmpInfo.ret_all_info = true;

    heldCfg = tmpCfg;
    tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);

    tmpValidity = vc->IsValid(vcm,tmpCfg,_env,_stats,tmpInfo,true,&call);
    tmpValidity = tmpValidity && !vcm->isInsideObstacle(tmpCfg,_env,tmpInfo);
    if ( tmpValidity ) {
      tmpValidity = tmpValidity && prevValidity;
      if ( !prevValidity ) // Extra Step TODO: Test if necessary
        prevValidity = true;
    }
    inBBX = tmpCfg.InBoundingBox(_env,_bb);
    if ( !inBBX ) { 
      if (_debug) cout << "ERROR: Fell out of BBX, error out... " << endl;
      return false;
    }
    stepSize += 1.0;
  }
  _cfg = tmpCfg;
  if (_debug) cout << "FINAL CfgType: " << _cfg << " steps: " << stepSize-1.0 << endl << call << "::END " << endl;
  return true;
}

bool PushFromInsideObstacle(MPProblem* _mp, CfgType& _cfg, Environment* _env, StatClass& _stats,
    string _vc, string _dm, bool _pExact, int _penetration, bool _debug, bool _positional) {
  return PushFromInsideObstacle(_mp, _cfg, _env, _env->GetBoundingBox(), _stats,
    _vc, _dm, _pExact, _penetration, _debug, _positional);
} 

//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//
bool PushCfgToMedialAxis(MPProblem* _mp, CfgType& _cfg, Environment* _env, shared_ptr<BoundingBox> _bb, 
    StatClass& _stats, string _vc, string _dm, bool _cExact, int _clearance, bool _useBBX, double _eps, 
    int _hLen, bool _debug, bool _positional) {
  // Initialization
  string call("MedialAxisUtility::PushCfgToMedialAxis");
  if (_debug) cout << call << endl << "Cfg: " << _cfg << " eps: " << _eps << endl;
  shared_ptr<DistanceMetricMethod>    dm    = _mp->GetDistanceMetric()->GetMethod(_dm);
  ValidityChecker<CfgType>*           vc    = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod>   vcm   = vc->GetVCMethod(_vc);
  shared_ptr<MultiBody>               robot = _env->GetMultiBody(_env->GetRobotIndex());

  // Variables
  Vector3D transDir, dif;
  CDInfo   tmpInfo,prevInfo;
  CfgType  transCfg, tmpCfg, heldCfg, tmpTransCfg;
  double   stepSize=0.0, cbStepSize=0.0, factor=0.0, posRes=_env->GetPositionRes(), oriRes=_env->GetOrientationRes();
  bool     inBBX=true, goodTmp=true, inside=vcm->isInsideObstacle(_cfg,_env,tmpInfo);
  int      nTicks;

  // Should already be in free space
  if ( inside ) 
    return false;

  // tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.ret_all_info = true;
  tmpCfg = _cfg;

  // Determine positional direction to move
  if ( CalculateCollisionInfo(_mp,_cfg,transCfg,_env,_stats,tmpInfo,_vc,_dm,_cExact,_clearance,0,_useBBX,_positional) ) {
    prevInfo = tmpInfo;
    if ( _cExact ) { // Exact uses workspace witness point for transDir
      Vector3D transDir = tmpInfo.robot_point - tmpInfo.object_point;
      for (int i=0; i<transCfg.DOF(); i++) {
        if ( i < transCfg.PosDOF() ) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i],2);
        } else transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg,posRes/sqrt(factor));
    } else { // Approx uses clearance CFG for transDir
      if (_debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg,transCfg,&nTicks,posRes,oriRes);
      transCfg.negative(transCfg);
      for (int i=transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if ( transCfg.GetSingleParam(i) > 0.5 )
          transCfg.SetSingleParam(i,transCfg.GetSingleParam(i) - 1.0, false);
      }
    }
    if (_debug) cout << "TRANS CFG: " << transCfg << endl;
  } else return false;

  // Initialize temp Info
  int posCnt=0, negCnt=0, zroCnt=0, stepsTaken, tcc=0;
  bool peakFound=false, wpMoved=false, checkBack=false, fellOut=false;;
  deque<double> segDists;
  deque<CfgType> segCfgs;
  double maxDist=0.0, errEps=0.00000001; // Arbitrary Value for errEps

  // Determine gap for medial axis
  while ( !peakFound && !wpMoved) {

    // Increment
    heldCfg = tmpCfg;
    if ( checkBack ) tmpCfg.multiply(transCfg,cbStepSize);
    else             tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);

    // Test for in BBX and inside obstacle
    inside = vcm->isInsideObstacle(tmpCfg,_env,tmpInfo);
    inBBX = tmpCfg.InBoundingBox(_env,_bb);
    bool tmpVal = (inside || !inBBX);
    if (_debug) VDAddTempCfg(tmpCfg, tmpVal);
    if (_debug) VDClearLastTemp();
    tcc++;

    // If inside obstacle or out of the bbx, step back
    if ( inside || !inBBX) {
      if ( !inBBX && !_useBBX ) 
        return false;
      if ( segCfgs.size() > 0 ) {
        fellOut = true;
        break;
      } else                       
        return false;
    } else {
      tmpInfo.ResetVars();
      tmpInfo.ret_all_info = true;

      // If tmp is valid, move on to next step
      if ( vc->IsValid(vcm,tmpCfg,_env,_stats,tmpInfo,true,&call) ) {
        if (_debug) cout << "TMP Cfg: " << tmpCfg;
        goodTmp = CalculateCollisionInfo(_mp,tmpCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                         _dm,_cExact,_clearance,0,_useBBX,_positional);
        if ( goodTmp ) {
          if ( _cExact ) { // If exact, check witness points
            Vector3D transDir = tmpInfo.object_point - prevInfo.object_point;
            if (_debug) cout << " obst: " << tmpInfo.nearest_obst_index;
            double tmpDist = 0.0;
            for (int i=0; i<transCfg.PosDOF(); i++)
              tmpDist += pow(transDir[i],2);
            tmpDist = sqrt(tmpDist);
            if (tmpDist > robot->GetBoundingSphereRadius()*2.0 ) { // TODO: might be better value
              if (_debug) cout << " WP moved: " << tmpDist;
              peakFound = true; 
              wpMoved=true;
            }
          }
          prevInfo = tmpInfo;
          if (_debug) cout << " clearance: " << tmpInfo.min_dist;
          ++stepsTaken;

          // Update clearance history distances
          if ( int(segDists.size()) > _hLen ) {
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
        else { // Couldn't calculate clearance info
          if (_debug) cout << "BROKE!" << endl;
          return false;
        }

        if ( !peakFound ) { // If no witness point success or approx, enumerate deltas to find peak
          maxDist = segDists[0]; posCnt=0; negCnt=0; zroCnt=0;
          for ( int i=0; i<int(segDists.size())-1; i++ ) {
            double tmp = segDists[i+1] - segDists[i];
            if      ( tmp >  errEps ) ++posCnt;
            else if ( tmp < -errEps ) ++negCnt;
            else                      ++zroCnt;
            maxDist = (maxDist>segDists[i+1])?maxDist:segDists[i+1];
          }
          if (_debug) cout << "  Pos/Zero/Neg counts: " << posCnt << "/" << zroCnt << "/" << negCnt << endl;
          if ( ( (negCnt > 0) && (negCnt > posCnt) ) || 
               ( (zroCnt > 0) && (zroCnt > posCnt) ) ) {
            if ( (posCnt < 1 && zroCnt < 1) ||
                 (posCnt < 1 && negCnt < 1) ){ // Only see negatives or zeros, check back
              --cbStepSize;
              --stepSize;
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
          } else { // No peak found
            checkBack = false;
          }
				}
        // Increment step size
        if ( !wpMoved && !peakFound )
          stepSize += 1.0;
      }
      else { // Else reduce step size or fall out of the loop
        return false;
      }
    }
  }

  // Check if there are enough cfgs
  if ( segCfgs.size() < 2 )
    return false;

  // Variables for modified binary search
  CfgType startCfg, midSCfg, midMCfg, midECfg, endingCfg;
  double gapDist, prevGap, lBound, uBound, middle, middleS, middleE;
  int attempts=0, maxAttempts=20, badPeaks=0, maxBadPeaks=10, peak;
  bool passed=false, peaked=false;
  vector<double> dists(5,0), deltas(4,0), distsFromMax(5,0);

  // Setup start, middle and end CfgTypes  
  startCfg  = (wpMoved)?(heldCfg):(segCfgs[0]);
  endingCfg = (wpMoved)?(tmpCfg):(segCfgs[segCfgs.size()-1]);
  ostringstream strng;
  strng << "Binary Cfgs: ";
  if (_debug) {
    VDComment(strng.str());
    VDAddTempCfg(startCfg, true);
    VDAddTempCfg(endingCfg, true);
  }

  if (_debug) cout << "Calculating Mid... stepSize/cbStepSize: " << stepSize << "/" << cbStepSize << endl;
  uBound = (fellOut)?(stepSize-1.0):(stepSize);
  if (wpMoved || fellOut) {
    lBound = uBound-1.0;
	} else {
    if (segCfgs.size() < (stepSize - cbStepSize))
      lBound = stepSize - segCfgs.size();
    else
      lBound = cbStepSize;
  }
  middle = (lBound+uBound)/2.0;
  if (_debug) cout << "Lower and upper bounds: " << lBound << "/" << middle << "/" << uBound  << endl;
  midMCfg.multiply(transCfg,middle);
  midMCfg.add(_cfg,midMCfg);
  
  if (_debug) VDClearComments();
  if (_debug) cout << "start/mid/end: " << endl << startCfg << endl << midMCfg << endl << endingCfg << endl;

  passed = CalculateCollisionInfo(_mp,startCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                  _dm,_cExact,_clearance,0,_useBBX,_positional);
  if ( !passed ) return false;
  dists[0] = tmpInfo.min_dist;

  passed = CalculateCollisionInfo(_mp,midMCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                  _dm,_cExact,_clearance,0,_useBBX,_positional);
  if ( !passed ) return false;
  dists[2] = tmpInfo.min_dist;

  passed = CalculateCollisionInfo(_mp,endingCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                  _dm,_cExact,_clearance,0,_useBBX,_positional);
  if ( !passed ) return false;
  dists[4] = tmpInfo.min_dist;

  middleS = (lBound+middle)/2.0;
  midSCfg.multiply(transCfg,middleS);
  midSCfg.add(_cfg,midSCfg);

  middleE = (middle+uBound)/2.0;
  midECfg.multiply(transCfg,middleE);
  midECfg.add(_cfg,midECfg);  

  if (_debug) {
    cout << "dists: ";
    for (int i=0; i<int(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }

  gapDist = dm->Distance(_env,startCfg,endingCfg);
  prevGap = gapDist;

  if (_debug) cout << "start/mids/end: " << endl << startCfg << endl << midSCfg 
                   << endl << midMCfg << endl << midECfg << endl << endingCfg << endl;

  do { // Modified Binomial search to find peaks

    if (_debug) {
      VDAddTempCfg(midMCfg, true);
      VDClearLastTemp();
    }

    // Update Cfgs
    attempts++;
    middle  = (lBound+uBound)/2.0;
    middleS = (lBound+middle)/2.0;
    middleE = (middle+uBound)/2.0;
    if (_debug) cout << "Bounds: " << lBound << "/" << middleS << "/" << middle << "/" << middleE << "/" << uBound << endl;
    midSCfg.multiply(transCfg,middleS);
    midSCfg.add(_cfg,midSCfg);
    midECfg.multiply(transCfg,middleE);
    midECfg.add(_cfg,midECfg);  

    passed = CalculateCollisionInfo(_mp,midSCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                    _dm,_cExact,_clearance,0,_useBBX,_positional);
    if ( !passed ) return false;
    dists[1] = tmpInfo.min_dist;

    passed = CalculateCollisionInfo(_mp,midECfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                    _dm,_cExact,_clearance,0,_useBBX,_positional); 
    if ( !passed ) return false;
    dists[3] = tmpInfo.min_dist;

    // Compute Deltas and Max Distance
    deltas.clear();
    distsFromMax.clear();
    maxDist = (dists[0]);
    if (_debug) cout << "Deltas: ";
    for ( int i=0; i<int(dists.size())-1; i++ ) {
      double tmp = dists[i+1] - dists[i];
      if (_debug) cout << " " << tmp;
      deltas.push_back(tmp);
      maxDist = (maxDist>dists[i+1])?maxDist:dists[i+1];
    } 
    for ( int i=0; i<int(dists.size()); i++ )
      distsFromMax.push_back(maxDist-dists[i]);

    if (_debug) cout << endl;

    // Determine Peak
    if ( deltas[0] > errEps && deltas[1] <= errEps && distsFromMax[1] < errEps) {
      peak = 1;
      endingCfg = midMCfg;
      midMCfg = midSCfg;
      dists[4] = dists[2];
      dists[2] = dists[1];
      uBound = middle;
    } 
    else if ( deltas[1] > errEps && deltas[2] <= errEps && distsFromMax[2] < errEps) {
      peak = 2;
      startCfg = midSCfg;
      endingCfg = midECfg;
      dists[0] = dists[1];
      dists[4] = dists[3];
      lBound = middleS;
      uBound = middleE;
    } 
    else if ( deltas[2] > errEps && deltas[3] <= errEps && distsFromMax[3] < errEps) {
      peak = 3;
      startCfg = midMCfg;
      midMCfg = midECfg;
      dists[0] = dists[2];
      dists[2] = dists[3];
      lBound = middle;
    } 
    else {
      if ( deltas[0] > errEps && deltas[1] > errEps && deltas[2] > errEps && deltas[3] > errEps) {
        peak = 3;
        startCfg = midMCfg;
        midMCfg = midECfg;
        dists[0] = dists[2];
        dists[2] = dists[3];
        lBound = middle;
      } 
      else if ( deltas[0] < -errEps && deltas[1] < -errEps && deltas[2] < -errEps && deltas[3] < -errEps) {
        peak = 1;
        endingCfg = midMCfg;
        midMCfg = midSCfg;
        dists[4] = dists[2];
        dists[2] = dists[1];
        uBound = middle;
      } 
      else {
        // No peak found, recalculate old, mid and new clearance
        badPeaks++;
        peak = -1;

        if ( CalculateCollisionInfo(_mp,startCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                    _dm,_cExact,_clearance,0,_useBBX,_positional) 
             && dists[0] >= tmpInfo.min_dist )
          dists[0] = tmpInfo.min_dist;

        if ( CalculateCollisionInfo(_mp,midMCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                    _dm,_cExact,_clearance,0,_useBBX,_positional) 
             && dists[2] >= tmpInfo.min_dist )
          dists[2] = tmpInfo.min_dist;

        if ( CalculateCollisionInfo(_mp,endingCfg,tmpTransCfg,_env,_stats,tmpInfo,_vc,
                                    _dm,_cExact,_clearance,0,_useBBX,_positional) 
             && dists[4] >= tmpInfo.min_dist )
          dists[4] = tmpInfo.min_dist;
      }
    }
    if (_debug) cout << " peak: " << peak << "  cfg: " << midMCfg;
    gapDist = dm->Distance(_env,startCfg,endingCfg);
    if (_debug) cout << " gap: " << gapDist << endl;
    if ( _eps >= gapDist )
      peaked = true;

  } while ( !peaked && attempts < maxAttempts && badPeaks < maxBadPeaks );

  if (_debug) {
    ostringstream strs;
    strs << "Final CFG";
    VDComment(strs.str());
    VDAddTempCfg(midMCfg, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
  }

  _cfg = midMCfg;
  if (_debug) cout << "FINAL Cfg: " << _cfg << " steps: " << stepSize << endl;
  return true;
}

bool PushCfgToMedialAxis(MPProblem* _mp, CfgType& _cfg, Environment* _env, StatClass& _stats,
    string _vc, string _dm, bool _cExact, int _clearance, bool _useBBX, double _eps, 
    int _hLen, bool _debug, bool _positional) {
  return PushCfgToMedialAxis(_mp, _cfg, _env, _env->GetBoundingBox(), _stats,
    _vc, _dm, _cExact, _clearance, _useBBX, _eps, _hLen, _debug, _positional);
}

//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
bool CalculateCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, 
    shared_ptr<BoundingBox> _bb, StatClass& _stats, CDInfo& _cdInfo, string _vc, string _dm, 
    bool _exact, int _clearance, int _penetration, bool _useBBX, bool _positional) {
  if ( _exact ) 
    return GetExactCollisionInfo(_mp,_cfg,_env,_bb,_stats,_cdInfo,_vc,_useBBX);
  else
    return GetApproxCollisionInfo(_mp,_cfg,_clrCfg,_env,_bb,_stats,_cdInfo,_vc,
                                  _dm,_clearance,_penetration,_useBBX,_positional);
}

bool CalculateCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string _vc, string _dm, bool _exact, int _clearance, int _penetration, 
    bool _useBBX, bool _positional) {
  return CalculateCollisionInfo(_mp, _cfg, _clrCfg, _env, _env->GetBoundingBox(), _stats, _cdInfo, 
    _vc, _dm, _exact, _clearance, _penetration, _useBBX,_positional);
}

//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
bool GetExactCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats,
    CDInfo& _cdInfo, string _vc, bool _useBBX) {
  // Setup Validity Checker
  string call("MedialAxisUtility::getExactCollisionInfo");
  ValidityChecker<CfgType>*         vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);
  _cdInfo.ResetVars();
  _cdInfo.ret_all_info = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  if ( !_cfg.InBoundingBox(_env,_bb) || !(vc->IsValid(vcm,_cfg,_env,_stats,_cdInfo,true,&call)) ) 
    return false;

  // If not using the bbx, done
  if ( !_useBBX )
    return true;

  // CfgType is now know as good, get BBX and ROBOT info
  boost::shared_ptr<MultiBody> robot = _env->GetMultiBody(_env->GetRobotIndex());
  std::pair<double,double> bbxRange;

  // Find closest point between robot and bbx, set if less than min dist from obstacles
  for(int m=0; m < robot->GetFreeBodyCount(); m++) {
    GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
    for(size_t j = 0 ; j < poly.vertexList.size() ; j++){
      for (int k=0; k<_cfg.PosDOF(); k++) { // For all positional DOFs
        bbxRange = _bb->GetRange(k);
        if ( (poly.vertexList[j][k]-bbxRange.first) < _cdInfo.min_dist ) {
          for (int l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.robot_point[l]  = poly.vertexList[j][l];
            _cdInfo.object_point[l] = poly.vertexList[j][l];
          }
          _cdInfo.object_point[k] = bbxRange.first; // Lower Bound
          _cdInfo.min_dist = poly.vertexList[j][k]-bbxRange.first;
          _cdInfo.nearest_obst_index = -(k*2);
        }
        if ( (bbxRange.second-poly.vertexList[j][k]) < _cdInfo.min_dist ) {
          for (int l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.robot_point[l]  = poly.vertexList[j][l];
            _cdInfo.object_point[l] = poly.vertexList[j][l];
          }
          _cdInfo.object_point[k] = bbxRange.second; // Upper Bound
          _cdInfo.min_dist = bbxRange.second-poly.vertexList[j][k];
          _cdInfo.nearest_obst_index = -(k*2+1);
        }
      }
    }
  }
  return (_cdInfo.min_dist>=0)?true:false;
} // END getExactCollisionInfo

bool GetExactCollisionInfo(MPProblem* _mp, CfgType& _cfg, Environment* _env, StatClass& _stats,
    CDInfo& _cdInfo, string _vc, bool _useBBX) {
  return GetExactCollisionInfo(_mp, _cfg, _env, _env->GetBoundingBox(), _stats,
    _cdInfo, _vc, _useBBX);
}

//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
bool GetApproxCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, 
    shared_ptr<BoundingBox> _bb, StatClass& _stats,CDInfo& _cdInfo, string _vc, string _dm, 
    int _clearance, int _penetration, bool _useBBX, bool _positional) {

  // Initialization
  string call = "MedialAxisUtility::GetApproxCollisionInfo";
  shared_ptr<DistanceMetricMethod>  dm  = _mp->GetDistanceMetric()->GetMethod(_dm);
  ValidityChecker<CfgType>*         vc  = _mp->GetValidityChecker();
  shared_ptr<ValidityCheckerMethod> vcm = vc->GetVCMethod(_vc);


  // Calculate MaxRange for dist calc
  double maxRange(0.0);
  for(int i=0; i< _cfg.PosDOF(); ++i) {
		std::pair<double,double> range = _env->GetBoundingBox()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) maxRange = tmpRange;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  if ( !_cfg.InBoundingBox(_env,_bb) )
    return false;

  _cdInfo.ResetVars();
  _cdInfo.ret_all_info = true;	

  bool initInside   = vcm->isInsideObstacle(_cfg,_env,_cdInfo);             // Initially Inside Obst
  bool initValidity = vc->IsValid(vcm,_cfg,_env,_stats,_cdInfo,true,&call); // Initial Validity
  bool initInBBX    = true; // _cfg.InBoundingBox(_env); Already Tested Above
  initValidity = initValidity && !initInside;
  if ( _useBBX ) 
    initValidity = (initValidity && initInBBX);

  // Setup Major Variables and Constants:
  CDInfo tmpInfo;
  Vector3D dif;
  int numRays;
  double posRes=_env->GetPositionRes(), oriRes=_env->GetOrientationRes();

  // Generate 'numRays' random directions at a distance 'dist' away from 'cfg'
  if ( initValidity ) 
    numRays = (initInside)?_penetration:_clearance;
  else                 
    numRays = _penetration;    

  vector<Cfg*> directions, candIn;
  vector<Cfg*> tick;
  vector<pair<Cfg*,double> > incr;
  vector<int> candTick;
  int nTicks;
  candIn.clear();

  // Setup translation rays
  for(int i=0; i<numRays; i++) {
    Cfg* tmp1 = _cfg.CreateNewCfg();
    tick.push_back(tmp1);
    Cfg* tmp2 = _cfg.CreateNewCfg();
    tmp2->GetRandomRay(posRes/maxRange, _env, dm, false);
    if ( _positional ) { // Use only positional dofs
      double factor=0.0;
      for (int i=0; i<tmp2->DOF(); i++) {
        if ( i < tmp2->PosDOF() ) {
          factor += pow(tmp2->GetSingleParam(i), 2);
        } else tmp2->SetSingleParam(i, 0.0);
      }
      tmp2->multiply(*tmp2, posRes/sqrt(factor), false);
    }
    tmp2->add(*tmp1,*tmp2);
    Cfg* tmp3 = _cfg.CreateNewCfg();
    tmp3->FindIncrement(*tmp1,*tmp2,&nTicks,posRes,oriRes);
    for (int i=tmp3->PosDOF(); i<tmp3->DOF(); i++) {
      if ( tmp3->GetSingleParam(i) > 0.5 )
        tmp3->SetSingleParam(i,tmp3->GetSingleParam(i) - 1.0, false);
    }
    incr.push_back(make_pair(tmp3, tmp3->OrientationMagnitude() +
          tmp3->PositionMagnitude()));
  }

  // Setup to step out along each direction:
  bool stateChangedFlag = false, currValidity, currInside, currInBBX;
  size_t lastLapIndex = -1, endIndex = -1;
  int iterations = 0, maxIterations=10000; // Arbitrary TODO: Smarter maxIter number
  CfgType tmpCfg;

  // Shoot out each ray to determine the candidates
  while ( !stateChangedFlag && iterations < maxIterations ) {
    iterations++;
    // For Each Ray
    for ( size_t i=0; i<incr.size(); ++i ) {
      tick[i]->Increment(*incr[i].first);
      currInside   = vcm->isInsideObstacle(*tick[i],_env,tmpInfo);
      currValidity = vc->IsValid(vcm,*tick[i],_env,_stats,tmpInfo,true,&call);
      currInBBX    = tick[i]->InBoundingBox(_env,_bb);
      currValidity = currValidity && !currInside;

      tmpCfg = *tick[i];

      if ( _useBBX ) 
        currValidity = (currValidity && currInBBX);
      if ( currValidity != initValidity ) { // If Validity State has changed
        if ( lastLapIndex != i ) {
          Cfg* candI = tick[i]->CreateNewCfg();
          candI->subtract(*candI,*incr[i].first);
          candIn.push_back(candI);
          candTick.push_back(i);
        } 
        if ( lastLapIndex == endIndex )  // Set lastLapIndex to first ray changing state
          lastLapIndex = (i==0)?(incr.size()-1):(i-1);
      }
      if (lastLapIndex == i)
        stateChangedFlag = true; // lastLapIndex == i, made full pass
      if ( stateChangedFlag ) break; // Once validity changes, exit loop
    } // End for
  } // End while

  if ( candIn.size() == 0)
    return false;

  double low=0.0, mid=0.5, high=1.0;
  Cfg* middleCfg = candIn[0]->CreateNewCfg();
  bool midInside, midValidity, midInBBX, foundOne;
  vector<bool> remove;

  // Binary search on candidates to find the best result
  while (candIn.size() > 1) {
    mid=(low+high)/2.0;
    remove.clear();
    foundOne=false;
    for ( size_t i=0; i<candIn.size(); i++ ) {

      middleCfg->multiply(*incr[candTick[i]].first,mid);
      middleCfg->add(*candIn[i],*middleCfg);

      midInside   = vcm->isInsideObstacle(*middleCfg,_env,tmpInfo);
      midValidity = vc->IsValid(vcm,*middleCfg,_env,_stats,tmpInfo,true,&call);
      midInBBX    = middleCfg->InBoundingBox(_env,_bb);
      midValidity = midValidity && !midInside;
      if ( _useBBX ) 
        midValidity = (midValidity && midInBBX);

      if ( midValidity != initValidity ) { // If Validity State has changed
        remove.push_back(false);
        foundOne = true;
      } else {
        remove.push_back(true);
      }
    }
    
    // If at least one good candidate is found, remove all known bad ones
    if ( foundOne ) {
      int offset=0;
      for ( size_t i=0; i<remove.size(); i++ ) {
        if ( remove[i] ) {
					candIn.erase(candIn.begin() + ((int)i-offset));
          candTick.erase(candTick.begin() + ((int)i-offset));
          offset++;
        }
      }
      high=mid;
    } else {
      low=mid;
    }
  }

  if ( initValidity ) { // Low may be initial cfg so keep looking
    while (low == 0.0) {
      mid=(low+high)/2.0;
      middleCfg->multiply(*incr[candTick[0]].first,mid);
      middleCfg->add(*candIn[0],*middleCfg);

      midInside   = vcm->isInsideObstacle(*middleCfg,_env,tmpInfo);
      midValidity = vc->IsValid(vcm,*middleCfg,_env,_stats,tmpInfo,true,&call);
      midInBBX    = middleCfg->InBoundingBox(_env,_bb);
      midValidity = midValidity && !midInside;
      if ( _useBBX ) 
        midValidity = (midValidity && midInBBX);
      if ( midValidity != initValidity ) // If Validity State has changed
        high=mid;
      else
        low=mid;
    }
    mid=low;
  } else { // Pushed out, high is all we need
    mid=high;
  }
  middleCfg->multiply(*incr[candTick[0]].first,mid);
  middleCfg->add(*candIn[0],*middleCfg);
  _clrCfg = *middleCfg;
  _cdInfo.min_dist = dm->Distance(_env,*middleCfg,_cfg);

  return (_cdInfo.min_dist < 0)?false:true;
} // END getApproxCollisionInfo

bool GetApproxCollisionInfo(MPProblem* _mp, CfgType& _cfg, CfgType& _clrCfg, Environment* _env, StatClass& _stats,
    CDInfo& _cdInfo, string _vc, string _dm, int _clearance, int _penetration, bool _useBBX, bool _positional) {
  return GetApproxCollisionInfo(_mp, _cfg, _clrCfg, _env, _env->GetBoundingBox(), _stats,
    _cdInfo, _vc, _dm, _clearance, _penetration, _useBBX, _positional);
}
