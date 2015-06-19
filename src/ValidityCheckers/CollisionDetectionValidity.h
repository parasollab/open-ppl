#ifndef COLLISION_DETECTION_VALIDITY_H_
#define COLLISION_DETECTION_VALIDITY_H_

#include "ValidityCheckerMethod.h"

#include "MPProblem/Environment.h"
#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/StaticMultiBody.h"

#include "Utilities/MetricUtils.h"

#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/VClipCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SolidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class CollisionDetectionValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod = NULL,
        bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);
    CollisionDetectionValidity(MPProblemType* _problem, XMLNode& _node);
    virtual ~CollisionDetectionValidity();

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);
    virtual bool IsInsideObstacle(const CfgType& _cfg);

    CollisionDetectionMethod* GetCDMethod() const {return m_cdMethod;}

  private:
    bool IsInCollision(CDInfo& _cdInfo, size_t _robotIndex, const string& _callName);
    bool IsInSelfCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
        const string& _callName);
    bool IsInObstCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
        shared_ptr<StaticMultiBody> _obst, const string& _callName);

    CollisionDetectionMethod* m_cdMethod;
    bool m_ignoreSelfCollision;
    size_t m_ignoreIAdjacentLinks;
};

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod,
    bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) :
  ValidityCheckerMethod<MPTraits>(), m_cdMethod(_cdMethod),
  m_ignoreSelfCollision(_ignoreSelfCollision),
  m_ignoreIAdjacentLinks(_ignoreIAdjacentLinks) {
    this->m_name = "CollisionDetection";
  }

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(MPProblemType* _problem, XMLNode& _node) :
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    this->m_name = "CollisionDetection";

    m_ignoreSelfCollision = _node.Read("ignoreSelfCollision", false, false,
        "Check for self collision");
    m_ignoreIAdjacentLinks = _node.Read("ignoreIAdjacentLinks", false, 1, 0,
        MAX_INT, "number of links to ignore for linkages");

    string cdLabel = _node.Read("method", true, "", "method");

    if(cdLabel == "BoundingSpheres")
      m_cdMethod = new BoundingSpheres();
    else if(cdLabel == "InsideSpheres")
      m_cdMethod = new InsideSpheres();
#ifdef USE_RAPID
    else if(cdLabel == "RAPID")
      m_cdMethod = new Rapid();
#endif
#ifdef USE_PQP
    else if(cdLabel == "PQP")
      m_cdMethod = new PQP();
    else if(cdLabel == "PQP_SOLID")
      m_cdMethod = new PQPSolid();
#endif
#ifdef USE_VCLIP
    else if(cdLabel == "VCLIP")
      m_cdMethod = new VClip();
#endif
#ifdef USE_SOLID
    else if(cdLabel == "SOLID")
      m_cdMethod = new Solid();
#endif
    else
      throw ParseException(_node.Where(),
          "Unknown collision detection library '" + cdLabel + "' requested.");
  }

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::
~CollisionDetectionValidity() {
  delete m_cdMethod;
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  this->GetStatClass()->IncCfgIsColl(_callName);

  //position environment
  _cfg.ConfigEnvironment();

  //check collision
#ifdef PMPCfgMultiRobot
  bool answerFromEnvironment = false;
  for(size_t i = 0; i < CfgType::m_numRobot; ++i)
    answerFromEnvironment |= IsInCollision(_cdInfo, i, _callName);
#else
  bool answerFromEnvironment = IsInCollision(_cdInfo, _cfg.GetRobotIndex(), _callName);
#endif

  _cfg.SetLabel("VALID", !answerFromEnvironment);
  return !answerFromEnvironment;
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsInCollision(CDInfo& _cdInfo,
    size_t _robotIndex,  const string& _callName) {

  /*Environment* env = this->GetEnvironment();

  size_t nMulti = env->GetUsableMultiBodyCount();
  shared_ptr<ActiveMultiBody> rob = env->GetActiveBody(_robotIndex);

  CDInfo localCDInfo;
  bool retVal = false;

  for(size_t i = 0; i < nMulti; i++) {
    if(i != _robotIndex) {
      // Note that the below call sets _cdInfo as needed
      bool collisionFound = IsInCollision(_cdInfo, rob, env->GetMultiBody(i), _callName);
      if (collisionFound && !_cdInfo.m_retAllInfo) {
        _cdInfo.m_collidingObstIndex = i;
        return true;
      }
      else  if(_cdInfo.m_retAllInfo) {  // store more info
        if(collisionFound && !retVal) {
          // m_collidingObstIndex is always the FIRST obstacle found in collision
          // m_nearestObstIndex is 'nearest' obstacle (colliding or not)
          localCDInfo.m_collidingObstIndex = i;
          retVal = true;
        }

        // Be certain that IsInCollision set _cdInfo.m_minDist
        // Check new mins against old, reset *_points if needed
        // Store everything in localCDInfo, copy back to _cdInfo at end of function.

        if (_cdInfo.m_minDist < localCDInfo.m_minDist) {
          localCDInfo.m_nearestObstIndex = i;
          localCDInfo.m_minDist = _cdInfo.m_minDist;
          localCDInfo.m_robotPoint = _cdInfo.m_robotPoint;
          localCDInfo.m_objectPoint = _cdInfo.m_objectPoint;
        } // end updating localCDInfo
      }
    }
    else {
      if(m_ignoreSelfCollision) {
        //robot self checking turned off
        if (_cdInfo.m_retAllInfo) {
          // localCDInfo should contain "all the info" across all objects
          // _cdInfo only contains info for the last one processed above
          _cdInfo = localCDInfo;
        }
        //The line below was commented because it does not make sense to reset the return value
        //to false whenever m_ignoreSelfCollision is true.
        //retVal = false;
      }
      else {
        // robot self checking. Warning: rob and _env->GetMultiBody(robot) may NOT be the same.
        if(rob->GetBodyCount() > 1 &&
            IsInCollision(_cdInfo, rob, rob, _callName)) {
          if(_cdInfo.m_retAllInfo) {
            // set stuff to indicate odd happenning
            _cdInfo.m_collidingObstIndex = -1;
            _cdInfo.m_minDist = MAX_DBL;
            _cdInfo.m_nearestObstIndex = -1;
            _cdInfo.m_robotPoint[0] = _cdInfo.m_robotPoint[1] = _cdInfo.m_robotPoint[2] = 0;
            _cdInfo.m_objectPoint[0] = _cdInfo.m_objectPoint[1] = _cdInfo.m_objectPoint[2] = 0;
          }
          return true;
        }
      }
    }
  }

  if (_cdInfo.m_retAllInfo) {
    // localCDInfo should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = localCDInfo;
  }

  return retVal;
  */

  CDInfo localCDInfo;
  bool retVal = false;

  //get robot
  Environment* env = this->GetEnvironment();
  shared_ptr<ActiveMultiBody> robot = env->GetActiveBody(_robotIndex);

  //self collision
  if(!m_ignoreSelfCollision && robot->GetBodyCount() > 1 &&
      IsInSelfCollision(_cdInfo, robot, _callName)) {
    return true;
  }

  //obstacle collisions
  size_t numObst = env->GetObstacleCount();
  for(size_t i = 0; i < numObst; ++i) {
    // Note that the below call sets _cdInfo as needed
    bool collisionFound = IsInObstCollision(_cdInfo, robot, env->GetStaticBody(i), _callName);
    if(collisionFound && !_cdInfo.m_retAllInfo) {
      _cdInfo.m_collidingObstIndex = i;
      return true;
    }
    else if(_cdInfo.m_retAllInfo) {  // store more info
      if(collisionFound && !retVal) {
        // m_collidingObstIndex is always the FIRST obstacle found in collision
        // m_nearestObstIndex is 'nearest' obstacle (colliding or not)
        localCDInfo.m_collidingObstIndex = i;
        retVal = true;
      }
    }
  }

  //finalize return information
  if (_cdInfo.m_retAllInfo) {
    // localCDInfo should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = localCDInfo;
  }

  return retVal;
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
    const string& _callName) {

  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  CollisionDetectionMethod::CDType tp = m_cdMethod->GetType();

  bool collision = m_cdMethod->IsInCollision(_rob, _rob, _cdInfo,
      m_ignoreIAdjacentLinks);

  switch(tp) {
    case CollisionDetectionMethod::CDType::Out:
      // Type Out: no collision sure; collision unsure.
    case CollisionDetectionMethod::CDType::Exact:
      // Type Exact: no collision sure; collision sure.
      return collision;

    case CollisionDetectionMethod::CDType::In:
      // Type In: no collision unsure; collision sure.
      // WARNING: If the Type is In, the result will always be true.
    default:
      return true;
  }
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInObstCollision(CDInfo& _cdInfo, shared_ptr<ActiveMultiBody> _rob,
    shared_ptr<StaticMultiBody> _obst, const string& _callName) {

  this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _callName);

  CollisionDetectionMethod::CDType tp = m_cdMethod->GetType();

  bool collision = m_cdMethod->IsInCollision(_rob, _obst, _cdInfo,
      m_ignoreIAdjacentLinks);

  switch(tp) {
    case CollisionDetectionMethod::CDType::Out:
      // Type Out: no collision sure; collision unsure.
    case CollisionDetectionMethod::CDType::Exact:
      // Type Exact: no collision sure; collision sure.
      return collision;

    case CollisionDetectionMethod::CDType::In:
      // Type In: no collision unsure; collision sure.
      // WARNING: If the Type is In, the result will always be true.
    default:
      return true;
  }
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsInsideObstacle(const CfgType& _cfg) {
#ifdef USE_PQP
  return PQPSolid().IsInsideObstacle(_cfg, this->GetEnvironment());
#else
  throw RunTimeException(WHERE, "Not implemented. Use PQP instead.");
#endif
}

#endif
