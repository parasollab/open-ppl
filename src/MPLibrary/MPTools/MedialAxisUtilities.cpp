#include "MedialAxisUtilities.h"

#include "MPLibrary/MPLibrary.h"

#include <ctgmath>
#include <deque>

/*--------------------------- Medial Axis Utility ----------------------------*/

MedialAxisUtility::
MedialAxisUtility(
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positionalDofsOnly, bool _debug,
    double _epsilon, size_t _historyLength) :
    ClearanceUtility(_vcLabel, _dmLabel,
      _exactClearance, _exactPenetration, _clearanceRays, _penetrationRays,
      _useBBX, _positionalDofsOnly, _debug),
    m_epsilon(_epsilon), m_historyLength(_historyLength) {
  this->SetName("MedialAxisUtility");
}


MedialAxisUtility::
MedialAxisUtility(XMLNode& _node) : ClearanceUtility(_node) {
  this->SetName("MedialAxisUtility");

  //These are now unused in MedialAxisUtilities.
//  m_epsilon = _node.Read("epsilon", false, m_epsilon, 0.0, 1.0,
//      "Epsilon-Close to the MA (fraction of the resolution)");
//  m_historyLength = _node.Read("historyLength", false, m_historyLength,
//      size_t(3), size_t(100), "History Length");

  // Note that all necessary initialization is done in ClearanceUtility
}


void
MedialAxisUtility::
Print(ostream& _os) const {
  ClearanceUtility::Print(_os);
  _os << "\tepsilon::" << m_epsilon << endl
      << "\thistory length::" << m_historyLength << endl;
}


bool
MedialAxisUtility::
PushToMedialAxis(Cfg& _cfg, const Boundary* const _b) {
  // Initialization
  string callee = this->GetNameAndLabel() + "::PushToMedialAxis";
  if(this->m_debug)
    cout << endl << callee << endl << "Being Pushed: " << _cfg;

  auto vcm = this->GetMPLibrary()->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars(true);//sets m_retAllInfo to true.

  // If invalid, push to the outside of the obstacle
  bool inCollision = !(vcm->IsValid(_cfg, tmpInfo, callee));

  if(this->m_debug)
    cout << "In-Collision: " << inCollision << endl;
  if(inCollision) {
    if(!PushFromInsideObstacle(_cfg, _b)) {
      if(this->m_debug)
        std::cout << callee << "Returning false from not being able to push "
            "from obstacle" << std::endl;
      return false;
    }
  }
  bool pushed = PushCfgToMedialAxisMidpointRule(_cfg, _b);

  if(!pushed) {
    if(this->m_debug)
      std::cout << callee << "Returning false from not being able to push to MA"
                << std::endl;
    return false;
  }

  if(this->m_debug)
    cout << "Successfully got MA sample: " << _cfg << endl << callee << "::END"
         << endl << endl;

  return true;
}


bool
MedialAxisUtility::
PushFromInsideObstacle(Cfg& _cfg, const Boundary* const _b) {
  string callee = this->GetNameAndLabel() + "::PushFromInsideObstacle";
  if(this->m_debug)
    cout << callee << endl << " Cfg: " << _cfg << endl;

  Cfg transCfg = _cfg;
  CDInfo tmpInfo;
  tmpInfo.ResetVars(true);//true so it returns all info.

  // Determine direction to move. Note that passing true to CollisionInfo means
  // all witnesses will be of opposite validity of the sample (_cfg).
  // This is what we need for pushing from the obstacle.
  if(!this->CollisionInfo(_cfg, transCfg, _b, tmpInfo, true)) {
    if(this->m_debug)
      std::cout << callee + " Returning false from failing to get a witness"
                << std::endl;
    return false;
  }

  // _cfg cannot equal transCfg here, as CollisionInfo would have returned false
  _cfg = transCfg;

  if(this->m_debug) {
    cout << "Clearance Cfg: " << transCfg << endl;
    cout << "FINAL Cfg: " << _cfg << endl << callee << "::END " << endl;
  }

  return true;
}


inline bool
MedialAxisUtility::
FuzzyVectorEquality(mathtool::Vector3d _v1, mathtool::Vector3d _v2,
                    double _tolerance) {
  return (_v1 - _v2).norm() <= _tolerance;
}


inline bool
MedialAxisUtility::
WitnessObstacleEquality(Cfg _cfg, CDInfo _firstWitnessInfo, CDInfo _secondWitnessInfo) {

  if (this->m_debug) {
    cout << "First Witness Nearest Obstacle Index: " << _firstWitnessInfo.m_nearestObstIndex << endl;
    cout << "Second Witness Nearest Obstacle Index: " << _secondWitnessInfo.m_nearestObstIndex << endl;
  }

  /*
  if(_firstWitnessInfo.m_nearestObstIndex == -1 && _secondWitnessInfo.m_nearestObstIndex == -1) {

    Vector3d firstNormalDirection = (_firstWitnessInfo.m_robotPoint - _firstWitnessInfo.m_objectPoint).normalize();
    Vector3d secondNormalDirection = (_secondWitnessInfo.m_robotPoint - _secondWitnessInfo.m_objectPoint).normalize();

    return ((firstNormalDirection - secondNormalDirection).norm() <= this->GetEnvironment()->GetPositionRes()
      * pow(2, _cfg.PosDOF()));;

  }
  */

  return (_firstWitnessInfo.m_nearestObstIndex == _secondWitnessInfo.m_nearestObstIndex);

}


bool
MedialAxisUtility::
PushCfgToMedialAxisMidpointRule(Cfg& _cfg, const Boundary* const _b) {
  //This function is using the premise of taking the midpoint of
  // 2 cfgs in C-Space and calling it the MA. Basically the witness point and
  // the normal to push the cfg along are found as usual, but this normal is
  // simply pushed along until another collision is found, then the midpoint
  // between the two is called the MA cfg.
  //This method means that the witness only needs to be found at the beginning,
  // then the cfg is pushed (checking collisions still) along that line. Only
  // doing the witness/ray shooting a single time is a HUGE benefit.
  //When using Exact clearance/witness finding, the iterations stop when the
  // witness changes. We can only do this in exact since the approximate witness
  // finding is extremely noise-prone and slow.
  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxisMidpointRule";
  if(this->m_debug)
    cout << callee << endl << "Cfg: " << _cfg << endl;

  auto vcm = this->GetMPLibrary()->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  CDInfo firstWitnessInfo;
  tmpInfo.ResetVars(true);//true so it returns all info.

  // Should already be in free space
  if(!vcm->IsValid(_cfg, tmpInfo, callee)) {
    if(this->m_debug)
      std::cout << callee <<": Returning false due to already invalid"
                << std::endl;
    return false;
  }

  //Get the first witness:
  Cfg firstWitnessCfg(_cfg.GetRobot());
  if(!this->CollisionInfo(_cfg, firstWitnessCfg, _b, tmpInfo, true)) {
    if(this->m_debug)
      std::cout << callee << ": Returning false from not being able to"
                             " find first witness" << std::endl;
    return false;
  }
  firstWitnessInfo = tmpInfo;

  Vector3d normalDirection = (tmpInfo.m_robotPoint - tmpInfo.m_objectPoint).normalize();

  //Find the unit normal:
//  Cfg unitDirectionToTickCfgAlong = _Cfg - firstWitnessCfg;
//  unitDirectionToTickCfgAlong /= (unitDirectionToTickCfgAlong.Magnitude());
  Cfg unitDirectionToTickCfgAlong;

  if(this->m_exactClearance)
    unitDirectionToTickCfgAlong = Cfg(normalDirection, _cfg.GetRobot());
  else {
    unitDirectionToTickCfgAlong = _cfg - firstWitnessCfg;
    unitDirectionToTickCfgAlong /= unitDirectionToTickCfgAlong.Magnitude();
  }

  if(this->m_debug)
    std::cout << callee << ": first normal directional cfg from witness = "
              << unitDirectionToTickCfgAlong << std::endl
              << "_cfg = " << _cfg << std::endl << "firstWitnessCfg = "
              << firstWitnessCfg << std::endl;

  //Move along normal until anther witness is found.
  //A better maxIterations might be in order, but this should be fine for now.
  const double maSearchResolution = this->GetEnvironment()->GetPositionRes()
                                    * this->m_maSearchResolutionFactor;
  const size_t maxIterations = _b->GetMaxDist()/this->m_rayTickResolution;
  Cfg magnitudeAndDirectionToTick = unitDirectionToTickCfgAlong
                                        * maSearchResolution;

  bool inBounds = true;
  bool valid = true;
  bool passed = false;
  Cfg tickedCfg = _cfg;
  mathtool::Vector3d firstWitnessVertex = tmpInfo.m_objectPoint;

  //The MA search loop:
  for(size_t tick = 1; tick < maxIterations; tick++) {
    //Tick the cfg:
    tickedCfg += magnitudeAndDirectionToTick;

    //Check if in bounds:
    inBounds = tickedCfg.InBounds(_b);
    if(!inBounds && !this->m_useBBX) {
      if(this->m_debug)
        std::cout << "Returning false from going OOB but not using BBX as "
                     "obstacle" << std::endl;
      return false;
    }

    if(this->m_exactClearance) {
      //Exact, so we want all CD info, specifically the witness point.
      // GetNearestVertexWitness will do this for us, including boundary witness
      // information, if BBX is active as an obstacle. It also returns the
      // IsValid() boolean for _cfg and puts all data in tmpInfo:
      valid = this->GetNearestVertexWitness(tickedCfg, tmpInfo, _b);

      if(!valid || !inBounds) {
        if(this->m_debug)
          std::cout << "Returning false from going invalid when looking for "
                       "second exact witness." << std::endl
                    << "_cfg = " << _cfg << std::endl << "first witness cfg = "
                    << firstWitnessCfg << std::endl << "unit tick direction = "
                    << unitDirectionToTickCfgAlong << std::endl << "Current "
                    "ticked cfg = " << tickedCfg << std::endl << std::endl;
        return false;
      }
      //For exact, we can quit as soon as we have a witness change.
      //I found some issues initially when  doing a pure equality, so check
      // the two witness vertices are sufficiently different.
      //static const double threshold = this->GetEnvironment()->GetPositionRes()
      //  * pow(2, _cfg->posDOF());

      // @TODO: check the witnesses are sufficiently different before it is
      // considered valid. Currently, using double the environment resolution is
      // a quick fix; however, we should be checking the relative difference
      // between the cfg and its witness to check if the witnesses are
      // sufficiently different from the first witness. It is not clear as to
      // how to do this
      //passed = !FuzzyVectorEquality(
      //          tmpInfo.m_objectPoint, firstWitnessVertex, threshold);
      passed = !WitnessObstacleEquality(_cfg, firstWitnessInfo, tmpInfo);
    }
    else {
      //Do the most simple validity check (not returning all info).
      valid = vcm->IsValid(tickedCfg, callee);
      passed = !inBounds || !valid;
    }

    if(passed) {
      if(this->m_debug && this->m_exactClearance)
        std::cout << callee << ": passed on tick " << tick << " with original "
                  << "witness vertex = " << firstWitnessVertex << " and second "
                  "witness vertex = " << tmpInfo.m_objectPoint
                  << std::endl << std::endl;
      break;// we have found the second and final witness, break out of the loop
    }
  }

  //check that it passed, and didn't just time out.
  if(!passed) {
    if(this->m_debug)
      std::cout << callee << " Returning false as no second witness could "
                              "be found" << std::endl;
    return false;
  }

  //If doing exact, then the MA sample is the midpoint of the two last ticks.
  if(this->m_exactClearance)
    firstWitnessCfg = tickedCfg - magnitudeAndDirectionToTick;

  //If doing approx, the first witness is already set, and the last witness is
  // simply the tickedCfg.

  //Find the midpoint of the two cfgs as the MA cfg:
  Cfg cfgMA = (firstWitnessCfg + tickedCfg)/2.0;

  //Now we have to double check that it's valid, since it's possible that it's
  // not, especially with increase in complexity of environment/narrow passages.
  if(!cfgMA.InBounds(_b) || !vcm->IsValid(cfgMA, callee)) {
    //It's either OOB or it's invalid. This pair of witness won't work for a
    // MA sample, so return false.
    if(this->m_debug)
      std::cout << callee << "Returning false due to invalid midpoint cfg"
                << std::endl;
    return false;
  }

  _cfg = cfgMA;

  if(this->m_debug) {
    VDComment("Final CFG");
    VDAddTempCfg(cfgMA, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
    cout << "FINAL Cfg: " << _cfg << endl;
  }

  return true;
}
