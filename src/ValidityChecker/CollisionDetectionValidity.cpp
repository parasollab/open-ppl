#include "CollisionDetectionValidity.h"
#include "MPProblem.h"
#include "Metrics.h"
#include "RapidCollisionDetection.h"
#include "PQPCollisionDetection.h"
#include "VClipCollisionDetection.h"
#include "SolidCollisionDetection.h"
#include "SpheresCollisionDetection.h"


CollisionDetectionValidity::
CollisionDetectionValidity() : ValidityCheckerMethod() {
  m_name = "CollisionDetection";
}

CollisionDetectionValidity::
CollisionDetectionValidity(CollisionDetectionMethod* _cdMethod, bool _ignoreSelfCollision, int _ignoreIAdjacentLinks) 
    : ValidityCheckerMethod(), m_cdMethod(_cdMethod), m_ignoreSelfCollision(_ignoreSelfCollision), m_ignoreIAdjacentLinks(_ignoreIAdjacentLinks) {
  m_name = "CollisionDetection";
}
 
CollisionDetectionValidity::
CollisionDetectionValidity(XMLNodeReader& _node, MPProblem* _problem) 
    : ValidityCheckerMethod(_node, _problem) {
  m_name = "CollisionDetection";
  
  m_ignoreSelfCollision = _node.boolXMLParameter("ignoreSelfCollision", false, false, "Check for self collision"); 
  m_ignoreIAdjacentLinks  = _node.numberXMLParameter("ignore_i_adjacent_links", false, 1, 0, 100, "number of links to ignore for linkages");
  
  std::string cd_label = _node.stringXMLParameter("method",true,"","method");
  bool method_found = false;
#ifdef USE_RAPID
  if (cd_label == "RAPID") {
    m_cdMethod = new Rapid();
    method_found = true;
  }
#endif
#ifdef USE_PQP
  if (cd_label == "PQP") {
    m_cdMethod = new PQP();
    method_found = true;
  } else if (cd_label == "PQP_SOLID") {
    m_cdMethod = new PQP_Solid();
    method_found = true;
  }
#endif
#ifdef USE_VCLIP
  if (cd_label == "VCLIP") {
    m_cdMethod = new VClip();
    method_found = true;
  }
#endif
#ifdef USE_SOLID
  if (cd_label == "SOLID") {
    m_cdMethod = new Solid();
    method_found = true;
  }
#endif
  if (cd_label == "BoundingSpheres") {
    m_cdMethod = new BoundingSpheres();
    method_found = true;
  }
  if (cd_label == "InsideSpheres") {
    m_cdMethod = new InsideSpheres();
    method_found = true;
  }
  if (!method_found) {
    std::cerr << "Unknown Collision Detection Library Label \"" << cd_label << "\"" << std::endl;
    exit(1);
  }
}

CollisionDetectionValidity::
~CollisionDetectionValidity() 
{}
 

bool
CollisionDetectionValidity::
IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, CDInfo& _cdInfo, std::string *_callName = NULL) {
  _stats.IncCfgIsColl(_callName);
  
  if(!_cfg.ConfigEnvironment(_env)) {
    _cfg.SetLabel("VALID", !true);
    //return true;
    return false;
  }
  
  bool Clear = (_callName) ? false : true; 
  if(!_callName)
    _callName = new std::string("isColl(e,s,cd,cdi,ep)");
  
  // after updating the environment(multibodies), Ask ENVIRONMENT
  // to check collision! (this is more nature.)
  
  bool answerFromEnvironment = IsInCollision(_env, _stats, _cdInfo, _callName);
#ifdef COLLISIONCFG
  if(answerFromEnvironment) {
    CollisionConfiguration[_cdInfo.m_collidingObstIndex].push_back(v);
  }
#endif
  
  if(Clear) 
    delete _callName;
  
  _cfg.SetLabel("VALID", !answerFromEnvironment); 
  return !answerFromEnvironment;
}


bool
CollisionDetectionValidity::
IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo, std::string *_callName) {
  int nmulti = _env->GetMultiBodyCount();
  int robot = _env->GetRobotIndex();
  shared_ptr<MultiBody> rob = _env->GetMultiBody(robot);
  
  CDInfo local_cd_info;
  bool ret_val = false;
  
  for (int i = 0; i < nmulti; i++) {
    if ( i != robot ) {
      // Note that the below call sets _cdInfo as needed
      bool collision_found = IsInCollision(_env, _stats, _cdInfo, rob, _env->GetMultiBody(i), _callName);
      if ( (collision_found) && ( ! _cdInfo.m_retAllInfo) ) {
        _cdInfo.m_collidingObstIndex = i;
        return true;
      } else  if (_cdInfo.m_retAllInfo) {  // store more info
        if ((collision_found) && (!ret_val)) {
          // m_collidingObstIndex is always the FIRST obstacle found in collision
          // m_nearestObstIndex is 'nearest' obstacle (colliding or not)
          local_cd_info.m_collidingObstIndex = i;
          ret_val = true;
        }

        // Be certain that IsInCollision set _cdInfo.m_minDist
        // Check new mins against old, reset *_points if needed
        // Store everything in local_cd_info, copy back to _cdInfo at end of function.

        if (_cdInfo.m_minDist < local_cd_info.m_minDist) {
          local_cd_info.m_nearestObstIndex = i;
          local_cd_info.m_minDist = _cdInfo.m_minDist;
          local_cd_info.m_robotPoint = _cdInfo.m_robotPoint;
          local_cd_info.m_objectPoint = _cdInfo.m_objectPoint;
        } // end updating local_cd_info
      }
    } else {
      if(m_ignoreSelfCollision){
        //robot self checking turned off
        if (_cdInfo.m_retAllInfo) {
          // local_cd_info should contain "all the info" across all objects
          // _cdInfo only contains info for the last one processed above
          _cdInfo = local_cd_info;
        }
        ret_val =false;
      } else {
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
    // local_cd_info should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = local_cd_info;
  }
  
  return ret_val;
} 


bool
CollisionDetectionValidity::
IsInCollision(Environment* _env, StatClass& _stats, CDInfo& _cdInfo,
    shared_ptr<MultiBody> _rob, shared_ptr<MultiBody> _obst, std::string *_callName) {
  int tp = m_cdMethod->GetType();
  // Type Out: no collision sure; collision unsure.
  if((tp == Out) && (m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks) == false)) {
    return false;
  }

  // Type In: no collision unsure; collision sure.
  if ((tp == In) && (m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks) == true)) {
    return true;
  }

  // Type Exact: no collision sure; collision sure.
  if(tp == Exact) {
    return m_cdMethod->IsInCollision(_rob, _obst, _stats, _cdInfo, _callName, m_ignoreIAdjacentLinks);
  }

  return true;
}

bool
CollisionDetectionValidity::
IsInsideObstacle(const Cfg& _cfg, Environment* _env, CDInfo& _cdInfo)
{
#ifdef USE_PQP
  return PQP_Solid().IsInsideObstacle(_cfg, _env);
#else
  cerr << "Recompile with PQP to use CollisionDetectionValidity::IsInsideObstacle" <<endl;
  exit(1);
#endif
}

