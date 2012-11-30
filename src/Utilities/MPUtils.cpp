#include "MPUtils.h"
#include "MetricUtils.h"
//#include "CollisionDetectionValidity.h"
//#include "ValidityChecker.h"
//#include "Sampler.h"
//#include "SamplerMethod.h"
//#include "LocalPlanners.h"
//#include "LocalPlannerMethod.h"
//#include "MapEvaluator.h"
//#include "MapEvaluationMethod.h"
//#include "Metrics.h"
//#include "MetricsMethod.h"
//#include "MPStrategy.h"
//#include "MPProblem.h"

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

// normally(gaussian) distributed random number generator implementation of Polar Box-Muller method.
double GRand(bool _reset) {
  double v1, v2, rsq;// Two independent random variables and r^2
  static bool hasNext = false;//flag that tells whether the next gaussian has been computed
  if(_reset)  { //reset hasNext to return the mean
    hasNext=false;
    return 0.0;
  }
  static double gset;
  //If the next value to return is not already computed, compute it and the one after that
  if(!hasNext) {
    do {
      v1 = 2*DRand() - 1.0;
      v2 = 2*DRand() - 1.0;
      rsq = v1*v1 + v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    double fac = sqrt(-2.0*log(rsq)/rsq);
    //Generates two gaussians and returns one and stores the other for another call 
    gset = v1*fac;
    hasNext = true;
    return v2*fac;
  } else {
    // Else return gset that was computed in the previous call
    hasNext = false;
    return gset;
  }
}

//Same as GRand but with a scales the output by _stdev and centers it around _mean
double GaussianDistribution(double _mean, double _stdev) {
  return GRand(false) * _stdev + _mean;
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

/*bool PushToMedialAxis(CfgType& _cfg, StatClass& _stats, const ClearanceParams& _cParams,
  double _epsilon, int _historyLength, shared_ptr<Boundary> _bb){

  // Initialization
  string call("MedialAxisUtility::PushToMedialAxis()");
  bool debug = _cParams.GetDebug();
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  string vcLabel = _cParams.m_vcLabel;
  if (debug)
    cout << endl << call << endl 
      << "Being Pushed: " << _cfg;
  shared_ptr<ValidityCheckerMethod> vcm = mp->GetValidityChecker()->GetMethod(vcLabel);
  bool inside, inCollision, found, pushed = true;
  CDInfo tmpInfo;
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  // If invalid, push to the outside of the obstacle
  inside = vcm->IsInsideObstacle(_cfg, env, tmpInfo);
  inCollision = !(vcm->IsValid(_cfg, env, _stats, tmpInfo, &call));
  if (debug) cout << " Inside/In-Collision: " << inside << "/" << inCollision << endl;
  if (inside || inCollision)
    pushed = PushFromInsideObstacle(_cfg, _stats, _cParams, _bb);
  if ( !pushed ) 
    return false;

  // Cfg is free, find medial axis
  found = PushCfgToMedialAxis(_cfg, _stats, _cParams, _epsilon, _historyLength, _bb);
  if ( !found )  { 
    if (debug) cout << "Not found!! ERR in pushtomedialaxis" << endl; 
    return false; 
  }

  if (debug) cout << "FINAL CfgType: " << _cfg << endl << call << "::END" << endl << endl;
  return true;
}

bool PushToMedialAxis(CfgType& _cfg, StatClass& _stats, const ClearanceParams& _cParams,
  double _epsilon, int _historyLength){
  return PushToMedialAxis(_cfg, _stats, _cParams, _epsilon, _historyLength,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
//***************************************************************//
// Push From Inside Obstacle                                     //
//   In this function, a cfg is known to be inside an obstacle.  //
// A direction is determined to move the cfg outside of the      //
// obstacle and is pushed till outside.                          //
//***************************************************************//

/*bool PushFromInsideObstacle(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, shared_ptr<Boundary> _bb){

  // Initialization
  string call("MedialAxisUtility::PushFromInsideObstacle");
  bool debug = _cParams.GetDebug();
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  string dmLabel = _cParams.m_dmLabel;
  string vcLabel = _cParams.m_vcLabel;
  bool pExact = _cParams.m_exactPenetration;
  if (debug) 
    cout << call << endl 
      << " CfgType: " << _cfg << endl;
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetMethod(dmLabel);
  shared_ptr<ValidityCheckerMethod> vcm = mp->GetValidityChecker()->GetMethod(vcLabel);

  // Variables
  CDInfo   tmpInfo;
  Vector3D transDir, dif;
  bool     inBBX;
  CfgType  endCfg=_cfg, tmpCfg=_cfg, heldCfg=_cfg, transCfg=_cfg;
  double   stepSize=1.0, factor;
  double   posRes=env->GetPositionRes();
  double   oriRes=env->GetOrientationRes(); 
  bool     initValidity=vcm->IsValid(_cfg,env,_stats,tmpInfo,&call);
  bool     tmpValidity=false, prevValidity=false;
  int      nTicks;

  // If in collision (using the exact case), must use approx
  pExact=pExact && initValidity; 

  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;

  //Determine direction to move
  if (CalculateCollisionInfo(_cfg,transCfg,_stats,tmpInfo,_cParams,_bb) ) {
    if ( pExact ) {
      Vector3D transDir = tmpInfo.m_objectPoint - tmpInfo.m_robotPoint;
      for (size_t i=0; i<transCfg.DOF(); i++) {
        if ( i < transCfg.PosDOF() ) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i],2);
        } else transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg,posRes/sqrt(factor));
    } else {
      if (debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg,transCfg,&nTicks,posRes,oriRes);
      for (size_t i=transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if ( transCfg.GetSingleParam(i) > 0.5 )
          transCfg.SetSingleParam(i,transCfg.GetSingleParam(i)-1.0, false);
      }
    }
    if (debug) cout << "TRANS CFG: " << transCfg << endl;
  } else return false;

  // Check if valid and outside obstacle
  while ( !tmpValidity ) {
    tmpInfo.ResetVars();
    tmpInfo.m_retAllInfo = true;

    heldCfg = tmpCfg;
    tmpCfg.multiply(transCfg,stepSize);
    tmpCfg.add(_cfg, tmpCfg);

    tmpValidity = vcm->IsValid(tmpCfg,env,_stats,tmpInfo,&call);
    tmpValidity = tmpValidity && !vcm->IsInsideObstacle(tmpCfg,env,tmpInfo);
    if ( tmpValidity ) {
      tmpValidity = tmpValidity && prevValidity;
      if ( !prevValidity ) // Extra Step TODO: Test if necessary
        prevValidity = true;
    }
    inBBX = tmpCfg.InBoundary(env,_bb);
    if ( !inBBX ) { 
      if (debug) cout << "ERROR: Fell out of BBX, error out... " << endl;
      return false;
    }
    stepSize += 1.0;
  }
  _cfg = tmpCfg;
  if (debug) cout << "FINAL CfgType: " << _cfg << " steps: " << stepSize-1.0 << endl << call << "::END " << endl;
  return true;
}

bool PushFromInsideObstacle(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams){
  return PushFromInsideObstacle(_cfg, _stats, _cParams,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
//***************************************************************//
// Push Cfg To Medial Axis                                       //
//   This function is to perform a regular push to medial axis   //
// algorithm stepping out at the resolution till the medial axis //
// is found, determined by the clearance.                        //
//***************************************************************//


/*bool PushCfgToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLength, shared_ptr<Boundary> _bb){
  // Initialization
  string call("MedialAxisUtility::PushCfgToMedialAxis");
  bool debug = _cParams.GetDebug();
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  string dmLabel = _cParams.m_dmLabel;
  string vcLabel = _cParams.m_vcLabel;
  bool cExact = _cParams.m_exactClearance;
  bool useBBX = _cParams.m_useBBX;
  if (debug) 
    cout << call << endl 
      << "Cfg: " << _cfg << " eps: " << _epsilon << endl;
  shared_ptr<DistanceMetricMethod>    dm    = mp->GetDistanceMetric()->GetMethod(dmLabel);
  shared_ptr<ValidityCheckerMethod>   vcm   = mp->GetValidityChecker()->GetMethod(vcLabel);
  shared_ptr<MultiBody>               robot = env->GetMultiBody(env->GetRobotIndex());
  
  // Variables
  Vector3D transDir, dif;
  CDInfo   tmpInfo,prevInfo;
  CfgType  transCfg, tmpCfg, heldCfg, tmpTransCfg;
  double   stepSize=0.0, cbStepSize=0.0, factor=0.0, posRes=env->GetPositionRes(), oriRes=env->GetOrientationRes();
  bool     inBBX=true, goodTmp=true, inside=vcm->IsInsideObstacle(_cfg,env,tmpInfo);
  int      nTicks;

  // Should already be in free space
  if ( inside ) 
    return false;

  // tmpInfo and Origin Setup
  tmpInfo.ResetVars();
  tmpInfo.m_retAllInfo = true;
  tmpCfg = _cfg;

  // Determine positional direction to move
  if ( CalculateCollisionInfo(_cfg,transCfg,_stats,tmpInfo,_cParams,_bb) ) {
    prevInfo = tmpInfo;
    if ( cExact ) { // Exact uses workspace witness point for transDir
      Vector3D transDir = tmpInfo.m_robotPoint - tmpInfo.m_objectPoint;
      for (size_t i=0; i<transCfg.DOF(); i++) {
        if ( i < transCfg.PosDOF() ) {
          transCfg.SetSingleParam(i, transDir[i]);
          factor += pow(transDir[i],2);
        } else transCfg.SetSingleParam(i, 0.0);
      }
      transCfg.multiply(transCfg,posRes/sqrt(factor));
    } else { // Approx uses clearance CFG for transDir
      if (debug) cout << "CLEARANCE CFG: " << transCfg << endl;
      transCfg.FindIncrement(_cfg,transCfg,&nTicks,posRes,oriRes);
      transCfg.negative(transCfg);
      for (size_t i=transCfg.PosDOF(); i<transCfg.DOF(); i++) {
        if ( transCfg.GetSingleParam(i) > 0.5 )
          transCfg.SetSingleParam(i,transCfg.GetSingleParam(i) - 1.0, false);
      }
    }
    if (debug) cout << "TRANS CFG: " << transCfg << endl;
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
    inside = vcm->IsInsideObstacle(tmpCfg,env,tmpInfo);
    inBBX = tmpCfg.InBoundary(env,_bb);
    bool tmpVal = (inside || !inBBX);
    if (debug) VDAddTempCfg(tmpCfg, tmpVal);
    if (debug) VDClearLastTemp();
    tcc++;

    // If inside obstacle or out of the bbx, step back
    if ( inside || !inBBX) {
      if ( !inBBX && !useBBX ) 
        return false;
      if ( segCfgs.size() > 0 ) {
        fellOut = true;
        break;
      } else                       
        return false;
    } else {
      tmpInfo.ResetVars();
      tmpInfo.m_retAllInfo = true;

      // If tmp is valid, move on to next step
      if ( vcm->IsValid(tmpCfg,env,_stats,tmpInfo,&call) ) {
        if (debug) cout << "TMP Cfg: " << tmpCfg;
        goodTmp = CalculateCollisionInfo(tmpCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb);
        if ( goodTmp ) {
          if ( cExact ) { // If exact, check witness points
            Vector3D transDir = tmpInfo.m_objectPoint - prevInfo.m_objectPoint;
            if (debug) cout << " obst: " << tmpInfo.m_nearestObstIndex;
            double tmpDist = 0.0;
            for (size_t i=0; i<transCfg.PosDOF(); i++)
              tmpDist += pow(transDir[i],2);
            tmpDist = sqrt(tmpDist);
            if (tmpDist > robot->GetBoundingSphereRadius()*2.0 ) { // TODO: might be better value
              if (debug) cout << " WP moved: " << tmpDist;
              peakFound = true; 
              wpMoved=true;
            }
          }
          prevInfo = tmpInfo;
          if (debug) cout << " clearance: " << tmpInfo.m_minDist;
          ++stepsTaken;

          // Update clearance history distances
          if ( int(segDists.size()) > _historyLength ) {
            segDists.erase(segDists.begin());
            segCfgs.erase(segCfgs.begin());
          }
          if ( checkBack ) {
            segDists.push_front(tmpInfo.m_minDist);
            segCfgs.push_front(tmpCfg);
          } else {
            segDists.push_back(tmpInfo.m_minDist);
            segCfgs.push_back(tmpCfg);
            checkBack=false;
          }
        }
        else { // Couldn't calculate clearance info
          if (debug) cout << "BROKE!" << endl;
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
          if (debug) cout << "  Pos/Zero/Neg counts: " << posCnt << "/" << zroCnt << "/" << negCnt << endl;
          if ( ( (negCnt > 0) && (negCnt > posCnt) ) || 
              ( (zroCnt > 0) && (zroCnt > posCnt) ) ) {
            if ( (posCnt < 1 && zroCnt < 1) ||
                (posCnt < 1 && negCnt < 1) ){ // Only see negatives or zeros, check back
              --cbStepSize;
              --stepSize;
              checkBack = true;
            } else {
              peakFound = true;
              if (debug) {
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
  double gapDist, lBound, uBound, middle, middleS, middleE;
  int attempts=0, maxAttempts=20, badPeaks=0, maxBadPeaks=10, peak;
  bool passed=false, peaked=false;
  vector<double> dists(5,0), deltas(4,0), distsFromMax(5,0);

  // Setup start, middle and end CfgTypes  
  startCfg  = (wpMoved)?(heldCfg):(segCfgs[0]);
  endingCfg = (wpMoved)?(tmpCfg):(segCfgs[segCfgs.size()-1]);
  ostringstream strng;
  strng << "Binary Cfgs: ";
  if (debug) {
    VDComment(strng.str());
    VDAddTempCfg(startCfg, true);
    VDAddTempCfg(endingCfg, true);
  }

  if (debug) cout << "Calculating Mid... stepSize/cbStepSize: " << stepSize << "/" << cbStepSize << endl;
  uBound = (fellOut)?(stepSize-1.0):(stepSize);
  if (wpMoved || fellOut) {
    lBound = uBound-1.0;
  } 
  else {
    if (segCfgs.size() < (stepSize - cbStepSize))
      lBound = stepSize - segCfgs.size();
    else
      lBound = cbStepSize;
  }
  middle = (lBound+uBound)/2.0;
  if (debug) cout << "Lower and upper bounds: " << lBound << "/" << middle << "/" << uBound  << endl;
  midMCfg.multiply(transCfg,middle);
  midMCfg.add(_cfg,midMCfg);

  if (debug) VDClearComments();
  if (debug) cout << "start/mid/end: " << endl << startCfg << endl << midMCfg << endl << endingCfg << endl;

  passed = CalculateCollisionInfo(startCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb);
  if ( !passed ) return false;
  dists[0] = tmpInfo.m_minDist;

  passed = CalculateCollisionInfo(midMCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb);
  if ( !passed ) return false;
  dists[2] = tmpInfo.m_minDist;

  passed = CalculateCollisionInfo(endingCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb);
  if ( !passed ) return false;
  dists[4] = tmpInfo.m_minDist;

  middleS = (lBound+middle)/2.0;
  midSCfg.multiply(transCfg,middleS);
  midSCfg.add(_cfg,midSCfg);

  middleE = (middle+uBound)/2.0;
  midECfg.multiply(transCfg,middleE);
  midECfg.add(_cfg,midECfg);  

  if (debug) {
    cout << "dists: ";
    for (int i=0; i<int(dists.size()); i++)
      cout << dists[i] << " ";
    cout << endl;
  }

  gapDist = dm->Distance(env,startCfg,endingCfg);

  if (debug) cout << "start/mids/end: " << endl << startCfg << endl << midSCfg 
    << endl << midMCfg << endl << midECfg << endl << endingCfg << endl;

  do { // Modified Binomial search to find peaks

    if (debug) {
      VDAddTempCfg(midMCfg, true);
      VDClearLastTemp();
    }

    // Update Cfgs
    attempts++;
    middle  = (lBound+uBound)/2.0;
    middleS = (lBound+middle)/2.0;
    middleE = (middle+uBound)/2.0;
    if (debug) cout << "Bounds: " << lBound << "/" << middleS << "/" << middle << "/" << middleE << "/" << uBound << endl;
    midSCfg.multiply(transCfg,middleS);
    midSCfg.add(_cfg,midSCfg);
    midECfg.multiply(transCfg,middleE);
    midECfg.add(_cfg,midECfg);  

    passed = CalculateCollisionInfo(midSCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb);
    if ( !passed ) return false;
    dists[1] = tmpInfo.m_minDist;

    passed = CalculateCollisionInfo(midECfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb); 
    if ( !passed ) return false;
    dists[3] = tmpInfo.m_minDist;

    // Compute Deltas and Max Distance
    deltas.clear();
    distsFromMax.clear();
    maxDist = (dists[0]);
    if (debug) cout << "Deltas: ";
    for ( int i=0; i<int(dists.size())-1; i++ ) {
      double tmp = dists[i+1] - dists[i];
      if (debug) cout << " " << tmp;
      deltas.push_back(tmp);
      maxDist = (maxDist>dists[i+1])?maxDist:dists[i+1];
    } 
    for ( int i=0; i<int(dists.size()); i++ )
      distsFromMax.push_back(maxDist-dists[i]);

    if (debug) cout << endl;

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

        if ( CalculateCollisionInfo(startCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb) 
            && dists[0] >= tmpInfo.m_minDist )
          dists[0] = tmpInfo.m_minDist;

        if ( CalculateCollisionInfo(midMCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb) 
            && dists[2] >= tmpInfo.m_minDist )
          dists[2] = tmpInfo.m_minDist;

        if ( CalculateCollisionInfo(endingCfg,tmpTransCfg,_stats,tmpInfo,_cParams,_bb) 
            && dists[4] >= tmpInfo.m_minDist )
          dists[4] = tmpInfo.m_minDist;
      }
    }
    if (debug) cout << " peak: " << peak << "  cfg: " << midMCfg;
    gapDist = dm->Distance(env,startCfg,endingCfg);
    if (debug) cout << " gap: " << gapDist << endl;
    if ( _epsilon >= gapDist )
      peaked = true;

  } while ( !peaked && attempts < maxAttempts && badPeaks < maxBadPeaks );

  if (debug) {
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
  if (debug) cout << "FINAL Cfg: " << _cfg << " steps: " << stepSize << endl;
  return true;
}

bool PushCfgToMedialAxis(CfgType& _cfg, StatClass& _stats,
  const ClearanceParams& _cParams, double _epsilon, int _historyLength){
  return PushCfgToMedialAxis(_cfg, _stats, _cParams, _epsilon, _historyLength,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
//*********************************************************************//
// Calculate Collision Information                                     //
//   This is a wrapper function for getting the collision information  //
// for the medial axis computation, calls either approx or exact       //
//*********************************************************************//
/*bool CalculateCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams, shared_ptr<Boundary> _bb){
  if ( _cParams.m_exactClearance ) 
    return GetExactCollisionInfo(_cfg,_stats,_cdInfo,_cParams,_bb);
  else
    return GetApproxCollisionInfo(_cfg,_clrCfg,_stats,_cdInfo,_cParams,_bb);
}

bool CalculateCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams){
  return CalculateCollisionInfo(_cfg,_clrCfg,_stats,_cdInfo,_cParams,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
//*********************************************************************//
// Get Exact Collision Information Function                            //
//   Determines the exact collision information by taking the validity //
// checker results against obstacles to the bounding box to get a      //
// complete solution                                                   //
//*********************************************************************//
/*bool GetExactCollisionInfo(CfgType& _cfg, StatClass& _stats, CDInfo& _cdInfo,
  const ClearanceParams& _cParams, shared_ptr<Boundary> _bb){

  // ClearanceParams variables
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  string vcLabel = _cParams.m_vcLabel;
  bool useBBX = _cParams.m_useBBX;
  
  // Setup Validity Checker
  string call("MedialAxisUtility::GetExactCollisionInfo");
  shared_ptr<ValidityCheckerMethod> vcm = mp->GetValidityChecker()->GetMethod(vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // If not in BBX or valid, return false (IsValid gets _cdInfo)
  if ( !_cfg.InBoundary(env,_bb) || !(vcm->IsValid(_cfg,env,_stats,_cdInfo,&call)) ) 
    return false;

  // If not using the bbx, done
  if ( !useBBX )
    return true;

  // CfgType is now know as good, get BBX and ROBOT info
  boost::shared_ptr<MultiBody> robot = env->GetMultiBody(env->GetRobotIndex());
  std::pair<double,double> bbxRange;

  // Find closest point between robot and bbx, set if less than min dist from obstacles
  for(int m=0; m < robot->GetFreeBodyCount(); m++) {
    GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
    for(size_t j = 0 ; j < poly.m_vertexList.size() ; j++){
      for (size_t k=0; k<_cfg.PosDOF(); k++) { // For all positional DOFs
        bbxRange = _bb->GetRange(k);
        if ( (poly.m_vertexList[j][k]-bbxRange.first) < _cdInfo.m_minDist ) {
          for (size_t l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.m_robotPoint[l]  = poly.m_vertexList[j][l];
            _cdInfo.m_objectPoint[l] = poly.m_vertexList[j][l];
          }
          _cdInfo.m_objectPoint[k] = bbxRange.first; // Lower Bound
          _cdInfo.m_minDist = poly.m_vertexList[j][k]-bbxRange.first;
          _cdInfo.m_nearestObstIndex = -(k*2);
        }
        if ( (bbxRange.second-poly.m_vertexList[j][k]) < _cdInfo.m_minDist ) {
          for (size_t l=0; l<_cfg.PosDOF(); l++) {// Save new closest point
            _cdInfo.m_robotPoint[l]  = poly.m_vertexList[j][l];
            _cdInfo.m_objectPoint[l] = poly.m_vertexList[j][l];
          }
          _cdInfo.m_objectPoint[k] = bbxRange.second; // Upper Bound
          _cdInfo.m_minDist = bbxRange.second-poly.m_vertexList[j][k];
          _cdInfo.m_nearestObstIndex = -(k*2+1);
        }
      }
    }
  }
  return (_cdInfo.m_minDist>=0)?true:false;
} // END getExactCollisionInfo

bool GetExactCollisionInfo(CfgType& _cfg, StatClass& _stats, CDInfo& _cdInfo,
  const ClearanceParams& _cParams){
  return GetExactCollisionInfo(_cfg,_stats,_cdInfo,_cParams,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
//*********************************************************************//
// Get Approximate Collision Information Function                      //
//   Calculate the approximate clearance using a series of rays. The   //
// specified number of rays are sent out till they change in validity. //
// The shortest ray is then considered the best calididate.            //
//*********************************************************************//
/*bool GetApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
    StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams, shared_ptr<Boundary> _bb){

  // ClearanceParams variables
  MPProblem* mp = _cParams.GetMPProblem();
  Environment* env = mp->GetEnvironment();
  string vcLabel = _cParams.m_vcLabel;
  string dmLabel = _cParams.m_dmLabel;
  bool useBBX = _cParams.m_useBBX;
  int clearance = _cParams.m_clearanceRays;
  int penetration = _cParams.m_penetrationRays;
  bool positional = _cParams.m_positional;

  // Initialization
  string call = "MedialAxisUtility::GetApproxCollisionInfo";
  shared_ptr<DistanceMetricMethod>  dm  = mp->GetDistanceMetric()->GetMethod(dmLabel);
  shared_ptr<ValidityCheckerMethod> vcm = mp->GetValidityChecker()->GetMethod(vcLabel);


  // Calculate MaxRange for dist calc
  double maxRange(0.0);
  for(size_t i=0; i< _cfg.PosDOF(); ++i) {
    std::pair<double,double> range = env->GetBoundary()->GetRange(i);
    double tmpRange = range.second-range.first;
    if(tmpRange > maxRange) 
      maxRange = tmpRange;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  if ( !_cfg.InBoundary(env,_bb) )
    return false;

  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;	

  bool initInside   = vcm->IsInsideObstacle(_cfg,env,_cdInfo);             // Initially Inside Obst
  bool initValidity = vcm->IsValid(_cfg,env,_stats,_cdInfo,&call); // Initial Validity
  initValidity = initValidity && !initInside;

  // Setup Major Variables and Constants:
  CDInfo tmpInfo;
  Vector3D dif;
  size_t numRays;
  double posRes=env->GetPositionRes(), oriRes=env->GetOrientationRes();

  // Generate 'numRays' random directions at a distance 'dist' away from 'cfg'
  if ( initValidity ) 
    numRays = (initInside)?penetration:clearance;
  else                 
    numRays = penetration;    

  vector<CfgType> directions, candIn, tick, incr;
  typedef vector<CfgType>::iterator CIT;
  vector<int> candTick;
  int nTicks;
  candIn.clear();

  // Setup translation rays
  for(size_t i=0; i<numRays; i++) {
    CfgType tmp1 = _cfg;
    tick.push_back(tmp1);
    CfgType tmp2;
    tmp2.GetRandomRay(posRes/maxRange, env, dm, false);
    if ( positional ) { // Use only positional dofs
      double factor=0.0;
      for (size_t j=0; j<tmp2.DOF(); j++) {
        if ( j < tmp2.PosDOF() ) {
          factor += pow(tmp2.GetSingleParam(j), 2);
        } 
        else 
          tmp2.SetSingleParam(j, 0.0);
      }
      tmp2.multiply(tmp2, posRes/sqrt(factor), false);
    }
    tmp2.add(tmp1, tmp2);
    CfgType tmp3;
    tmp3.FindIncrement(tmp1, tmp2, &nTicks, posRes, oriRes);
    for (size_t j = tmp3.PosDOF(); j < tmp3.DOF(); j++) {
      if ( tmp3.GetSingleParam(j) > 0.5 )
        tmp3.SetSingleParam(j, tmp3.GetSingleParam(j) - 1.0, false);
    }
    incr.push_back(tmp3);
  }

  // Setup to step out along each direction:
  bool stateChangedFlag = false, currValidity, currInside, currInBBX;
  size_t lastLapIndex = -1;
  size_t iterations = 0, maxIterations=10000; // Arbitrary TODO: Smarter maxIter number

  // Shoot out each ray to determine the candidates
  while ( !stateChangedFlag && iterations < maxIterations ) {
    iterations++;
    // For Each Ray
    size_t i = 0;
    for (CIT incrIT = incr.begin(), tickIT = tick.begin(); 
        incrIT!=incr.end() || tickIT!=tick.end(); ++incrIT, ++tickIT) {
      tickIT->Increment(*incrIT);
      currInside   = vcm->IsInsideObstacle(*tickIT,env,tmpInfo);
      currValidity = vcm->IsValid(*tickIT,env,_stats,tmpInfo,&call);
      currInBBX    = tickIT->InBoundary(env,_bb);
      currValidity = currValidity && !currInside;

      if ( useBBX ) 
        currValidity = (currValidity && currInBBX);

      if ( currValidity != initValidity ) { // If Validity State has changed
        if ( lastLapIndex != i ) {
          CfgType candI;
          candI.negative(*incrIT);
          candI.add(*tickIT, candI);
          candIn.push_back(candI);
          candTick.push_back(i);
        } 
        if ( lastLapIndex == (size_t)-1 )  // Set lastLapIndex to first ray changing state
          lastLapIndex = (i==0)?(incr.size()-1):(i-1);
      }
      if (lastLapIndex == i){
        stateChangedFlag = true; // lastLapIndex == i, made full pass
        break; // Once validity changes, exit loop
      }
      i++;
    } // End for
  } // End while

  if ( candIn.size() == 0)
    return false;

  double low=0.0, mid=0.5, high=1.0;
  CfgType middleCfg;
  bool midInside, midValidity, midInBBX, foundOne;
  vector<bool> remove;

  // Binary search on candidates to find the best result
  while (candIn.size() > 1) {
    mid=(low+high)/2.0;
    remove.clear();
    foundOne=false;
    for ( size_t i=0; i<candIn.size(); i++ ) {
      middleCfg.multiply(incr[candTick[i]], mid);
      middleCfg.add(candIn[i], middleCfg);
      midInside   = vcm->IsInsideObstacle(middleCfg,env,tmpInfo);
      midValidity = vcm->IsValid(middleCfg,env,_stats,tmpInfo,&call);
      midInBBX    = middleCfg.InBoundary(env,_bb);
      midValidity = midValidity && !midInside;
      if ( useBBX ) 
        midValidity = (midValidity && midInBBX);
    
      if ( midValidity != initValidity ) { // If Validity State has changed
        remove.push_back(false);
        foundOne = true;
      } 
      else {
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
    } 
    else {
      low=mid;
    }
  }

  if ( initValidity ) { // Low may be initial cfg so keep looking
    while (low == 0.0) {
      mid=(low+high)/2.0;
      middleCfg.multiply(incr[candTick[0]],mid);
      middleCfg.add(candIn[0], middleCfg);

      midInside   = vcm->IsInsideObstacle(middleCfg,env,tmpInfo);
      midValidity = vcm->IsValid(middleCfg,env,_stats,tmpInfo,&call);
      midInBBX    = middleCfg.InBoundary(env,_bb);
      midValidity = midValidity && !midInside;
      if ( useBBX ) 
        midValidity = (midValidity && midInBBX);
      if ( midValidity != initValidity ) // If Validity State has changed
        high=mid;
      else
        low=mid;
    }
    mid=low;
  } 
  else { // Pushed out, high is all we need
    mid=high;
  }
  middleCfg.multiply(incr[candTick[0]],mid);
  middleCfg.add(candIn[0], middleCfg);
  _clrCfg = middleCfg;
  _cdInfo.m_minDist = dm->Distance(env, middleCfg, _cfg);

  return (_cdInfo.m_minDist < 0)?false:true;
} // END getApproxCollisionInfo

bool GetApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
  StatClass& _stats, CDInfo& _cdInfo, const ClearanceParams& _cParams){
  return GetApproxCollisionInfo(_cfg,_clrCfg,_stats,_cdInfo,_cParams,
    _cParams.GetMPProblem()->GetEnvironment()->GetBoundary());
}
*/
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
  bool PtInTriangle 
(const Point2d& _A, const Point2d& _B, const Point2d& _C,const Point2d & _P)
{
  // FIRST CHECK THE SIGN OF THE Z-COMPONENT OF THE NORMAL BY CALCULATING
  // THE CROSS-PRODUCT (ABxBC). THIS WILL DETERMINE THE ORDERING OF THE
  // VERTICES. IF NEGATIVE, VERTICES ARE CLOCKWISE ORDER; OTHERWISE CCW.
  // THEN EVALUATE SIGN OF Z-COMPONENTS FOR ABxAP, BCxBP, and CAxCP TO
  // DETERMINE IF P IS IN "INSIDE" HALF-SPACE FOR EACH EDGE IN TURN ("INSIDE"
  // IS DETERMINED BY SIGN OF Z OF NORMAL (VERTEX ORDERING).
  // NOTE: FULL CROSS-PRODS ARE NOT REQUIRED; ONLY THE Z-COMPONENTS
  Vector2d dAB=_B-_A, dBC=_C-_B;  // "REPEATS"
  if ((dAB[0]*dBC[1]-dAB[1]*dBC[0]) < 0) // CW
  {
    if (dAB[0]*(_P[1]-_A[1]) >= dAB[1]*(_P[0]-_A[0])) return false;           // ABxAP
    if (dBC[0]*(_P[1]-_B[1]) >= dBC[1]*(_P[0]-_B[0])) return false;           // BCxBP
    if ((_A[0]-_C[0])*(_P[1]-_C[1]) >= (_A[1]-_C[1])*(_P[0]-_C[0])) return false; // CAxCP
  }
  else // CCW
  {
    if (dAB[0]*(_P[1]-_A[1]) < dAB[1]*(_P[0]-_A[0])) return false;           // ABxAP
    if (dBC[0]*(_P[1]-_B[1]) < dBC[1]*(_P[0]-_B[0])) return false;           // BCxBP
    if ((_A[0]-_C[0])*(_P[1]-_C[1]) < (_A[1]-_C[1])*(_P[0]-_C[0])) return false; // CAxCP
  }
  return true; // "INSIDE" EACH EDGE'S IN-HALF-SPACE (PT P IS INSIDE TRIANGLE)
}

//----------------------------------------------------------------------------
// CHECKS IF 2D POINT P IS IN TRIANGLE ABC. RETURNS 1 IF IN, 0 IF OUT
//   uses barycentric coordinates to compute this and return the uv-coords
//   for potential usage later
//----------------------------------------------------------------------------
bool PtInTriangle
(const Point2d& _A, const Point2d& _B, const Point2d& _C,const Point2d& _P,
 double& _u, double& _v) {
  // Compute vectors        
  Vector2d v0 = _C - _A;
  Vector2d v1 = _B - _A;
  Vector2d v2 = _P - _A;

  // Compute dot products
  double dot00 = v0*v0;
  double dot01 = v0*v1;
  double dot02 = v0*v2;
  double dot11 = v1*v1;
  double dot12 = v1*v2;

  // Compute barycentric coordinates
  double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  _u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  _v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (_u > 0) && (_v > 0) && (_u + _v < 1);
}

Point3d GetPtFromBarycentricCoords 
(const Point3d& _A, const Point3d& _B, const Point3d& _C, double _u, double _v) {
  // P = A + u * (C - A) + v * (B - A)
  Point3d p = _A + (_u*(_C - _A)) + (_v*(_B - _A));
  return p;
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
