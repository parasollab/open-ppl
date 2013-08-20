#ifndef COLLISIONDETECTIONVALIDITY_H_
#define COLLISIONDETECTIONVALIDITY_H_

#include "MPProblem/Environment.h"
#include "Utilities/MetricUtils.h"
#include "ValidityCheckerMethod.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"
#include "ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/VClipCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SolidCollisionDetection.h"
#include "ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

template<class MPTraits>
class CollisionDetectionValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;

    CollisionDetectionValidity();
    CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod, bool _ignoreSelfCollision = false, int _ignoreIAdjacentLinks = 1);
    CollisionDetectionValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~CollisionDetectionValidity();

    virtual bool IsValidImpl(CfgType& _cfg, Environment* _env,
        StatClass& _stats, CDInfo& _cdInfo,
        string* _callName);
    virtual bool IsInsideObstacle(const CfgType& _cfg, Environment* _env, CDInfo& _cdInfo);

    cd_predefined GetCDType() const { return m_cdMethod->GetCDType(); }

  private:
    bool IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
        shared_ptr<MultiBody> _rob, shared_ptr<MultiBody> _obst, string* _callName);
    bool IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
        size_t _robotIndex, string* _callName);

    CollisionDetectionMethod* m_cdMethod;
    bool m_ignoreSelfCollision;
    int m_ignoreIAdjacentLinks;
};

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::CollisionDetectionValidity() :
  ValidityCheckerMethod<MPTraits>(),
  m_cdMethod(NULL),
  m_ignoreSelfCollision(false),
  m_ignoreIAdjacentLinks(0) {
  this->m_name = "CollisionDetection";
}

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod, bool _ignoreSelfCollision, int _ignoreIAdjacentLinks)
  : ValidityCheckerMethod<MPTraits>(), m_cdMethod(_cdMethod), m_ignoreSelfCollision(_ignoreSelfCollision), m_ignoreIAdjacentLinks(_ignoreIAdjacentLinks) {
    this->m_name = "CollisionDetection";
  }

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::CollisionDetectionValidity(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node)
  : ValidityCheckerMethod<MPTraits>(_problem, _node) {
    this->m_name = "CollisionDetection";

    m_ignoreSelfCollision = _node.boolXMLParameter("ignoreSelfCollision", false, false, "Check for self collision");
    m_ignoreIAdjacentLinks  = _node.numberXMLParameter("ignore_i_adjacent_links", false, 1, 0, 100, "number of links to ignore for linkages");

    string cdLabel = _node.stringXMLParameter("method",true,"","method");

    if (cdLabel == "BoundingSpheres")
      m_cdMethod = new BoundingSpheres();
    else if (cdLabel == "InsideSpheres")
      m_cdMethod = new InsideSpheres();
#ifdef USE_RAPID
    else if (cdLabel == "RAPID")
      m_cdMethod = new Rapid();
#endif
#ifdef USE_PQP
    else if (cdLabel == "PQP")
      m_cdMethod = new PQP();
    else if (cdLabel == "PQP_SOLID")
      m_cdMethod = new PQPSolid();
#endif
#ifdef USE_VCLIP
    else if (cdLabel == "VCLIP")
      m_cdMethod = new VClip();
#endif
#ifdef USE_SOLID
    else if (cdLabel == "SOLID")
      m_cdMethod = new Solid();
#endif
    else {
      cerr << "Unknown Collision Detection Library Label \"" << cdLabel << "\"" << endl;
      exit(1);
    }
  }

template<class MPTraits>
CollisionDetectionValidity<MPTraits>::~CollisionDetectionValidity() {
  delete m_cdMethod;
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsValidImpl(CfgType& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, string* _callName = NULL) {
  _stats.IncCfgIsColl(_callName);

  if(!_cfg.ConfigEnvironment(_env)) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  bool clear = (_callName) ? false : true;
  if(!_callName)
    _callName = new string("isColl(e,s,cd,cdi,ep)");

  // after updating the environment(multibodies), Ask ENVIRONMENT
  // to check collision! (this is more natural)

  bool answerFromEnvironment = IsInCollision(_env, _stats, _cdInfo, _cfg.GetRobotIndex(), _callName);

  if(clear)
    delete _callName;

  _cfg.SetLabel("VALID", !answerFromEnvironment);
  return !answerFromEnvironment;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
    size_t _robotIndex,  string* _callName) {

  size_t nMulti = _env->GetUsableMultiBodyCount();
  shared_ptr<MultiBody> rob = _env->GetMultiBody(_robotIndex);

  CDInfo localCDInfo;
  bool retVal = false;

  for (size_t i = 0; i < nMulti; i++) {
    if ( i != _robotIndex ) {
      // Note that the below call sets _cdInfo as needed
      bool collisionFound = IsInCollision(_env, _stats, _cdInfo, rob, _env->GetMultiBody(i), _callName);
      if ( (collisionFound) && (!_cdInfo.m_retAllInfo) ) {
        _cdInfo.m_collidingObstIndex = i;
        return true;
      }
      else  if (_cdInfo.m_retAllInfo) {  // store more info
        if ((collisionFound) && (!retVal)) {
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
      if(m_ignoreSelfCollision){
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
        if ( (rob->GetBodyCount() > 1) &&
            (IsInCollision(_env, _stats, _cdInfo, rob, rob, _callName)) ) {
          if (_cdInfo.m_retAllInfo) {
            // set stuff to indicate odd happenning
            _cdInfo.m_collidingObstIndex = -1;
            _cdInfo.m_minDist = MaxDist;
            _cdInfo.m_nearestObstIndex = -1;
            _cdInfo.m_robotPoint[0] = _cdInfo.m_robotPoint[1] = _cdInfo.m_robotPoint[2] = 0;
            _cdInfo.m_objectPoint[0] = _cdInfo.m_objectPoint[1] = _cdInfo.m_objectPoint[2] = 0;
          }
          return true;
        }
      }
    } // end  if-else i == robot
  } // end for i

  if (_cdInfo.m_retAllInfo) {
    // localCDInfo should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = localCDInfo;
  }

  return retVal;
}


template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
    shared_ptr<MultiBody> _rob, shared_ptr<MultiBody> _obst, string* _callName) {
  CollisionDetectionMethod::CDType tp = m_cdMethod->GetType();
  // Type Out: no collision sure; collision unsure.
  if((tp == CollisionDetectionMethod::Out) && (m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks) == false)) {
    return false;
  }

  // Type In: no collision unsure; collision sure.
  // WARNING: If the Type is In, the result will always be true.
  if ((tp == CollisionDetectionMethod::In) && (m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks) == true)) {
    return true;
  }

  // Type Exact: no collision sure; collision sure.
  if(tp == CollisionDetectionMethod::Exact) {
    return m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks);
  }

  return true;
}

template<class MPTraits>
bool
CollisionDetectionValidity<MPTraits>::IsInsideObstacle(const CfgType& _cfg, Environment* _env, CDInfo& _cdInfo) {
#ifdef USE_PQP
  return PQPSolid().IsInsideObstacle(_cfg, _env);
#else
  cerr << "Recompile with PQP to use CollisionDetectionValidity::IsInsideObstacle" << endl;
  exit(1);
#endif
}

#endif
