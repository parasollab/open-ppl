#ifndef _COLLISION_DETECTION_VALIDITY_HPP_
#define _COLLISION_DETECTION_VALIDITY_HPP_

//////////////////////////////////////////////////////////////////////////////////////////
#include "ValidityCheckerMethod.hpp"
#include "CollisionDetection.h"
#include "MetricUtils.h"
#include <string>
//////////////////////////////////////////////////////////////////////////////////////////

template<typename CFG>
class CollisionDetectionValidity : public ValidityCheckerMethod 
{
public:
  CollisionDetectionValidity() { }
  CollisionDetectionValidity(vector<CollisionDetectionMethod*> selected, CollisionDetection* cd) : m_selected(selected), m_cd(cd) {}
  CollisionDetectionValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem);   
  virtual ~CollisionDetectionValidity() 
  {
    delete m_cd;
    m_cd = NULL;

    std::vector<CollisionDetectionMethod*>::iterator delIter;
    for (delIter = m_selected.begin(); delIter != m_selected.end(); delIter = m_selected.erase(delIter)){
      delete *delIter;
      *delIter = NULL;
    }
  }
  
  virtual bool IsValid(Cfg& _cfg, Environment* env, 
		       StatClass& Stats, CDInfo& _cdInfo, 
		       bool enablePenetration, std::string *pCallName);    
	virtual  bool isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo); 
  
private:
  std::vector<CollisionDetectionMethod*> m_selected;
  CollisionDetection* m_cd;
  bool ignoreSelfCollision;
  
  bool IsInCollision(Environment* env, StatClass& Stats, CDInfo& _cdInfo,
		     shared_ptr<MultiBody> rob, shared_ptr<MultiBody> obst, std::string *pCallName);
  
  bool IsInCollision(Environment* env, StatClass& Stats, CDInfo& _cdInfo, 
		     shared_ptr<MultiBody> lineRobot, bool enablePenetration, std::string *pCallName);

  int ignore_i_adjacent_links;

};


template<typename CFG>
CollisionDetectionValidity<CFG>::
CollisionDetectionValidity(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  ValidityCheckerMethod(in_Node, in_pProblem) 
{
  std::vector<CollisionDetectionMethod*>::iterator I;
  for(I=m_selected.begin(); I!=m_selected.end(); ++I)
    delete *I;
  m_selected.clear(); 
ignoreSelfCollision = in_Node.boolXMLParameter("ignoreSelfCollision", false, false, "Check for self collision"); 
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection(); 
  std::string cd_label = in_Node.stringXMLParameter("method",true,"","method");
  ignore_i_adjacent_links  = in_Node.numberXMLParameter("ignore_i_adjacent_links", false, int(1),
						int(0), int(100),
					       "number of links to ignore for linkages");
  

  if (cd_label == "RAPID") {
    m_selected.push_back(cd->GetRAPID());	    
  } else if (cd_label == "PQP") {
    m_selected.push_back(cd->GetPQP());
  } else if (cd_label == "PQP_SOLID") {
    m_selected.push_back(cd->GetPQP_SOLID());
  } else if (cd_label == "VCLIP") {
    m_selected.push_back(cd->GetVCLIP());
  } else if (cd_label == "SOLID") {
    m_selected.push_back(cd->GetSOLID());
  } else {
    std::cerr << "Unknown Label" << std::endl;
  }
  m_cd = new CollisionDetection(m_selected);
}


template<typename CFG>
bool
CollisionDetectionValidity<CFG>::
IsValid(Cfg& _cfg, Environment* env, StatClass& Stats, CDInfo& _cdInfo, 
	bool enablePenetration, std::string *pCallName = NULL) 
{
  Stats.IncCfgIsColl(pCallName);
  
  if(!_cfg.ConfigEnvironment(env))
    {
      _cfg.SetLabel("VALID",!true);
      //return true;
      return false;
    }
  bool Clear = (pCallName) ? false : true; 
  if( !pCallName )
    pCallName = new std::string("isColl(e,s,cd,cdi,ep)");
  
  // after updating the environment(multibodies), Ask ENVIRONMENT
  // to check collision! (this is more nature.)
  
  bool answerFromEnvironment = IsInCollision(env, Stats, _cdInfo, shared_ptr<MultiBody>(), true, pCallName);
#ifdef COLLISIONCFG
  if (answerFromEnvironment) {
    CollisionConfiguration[_cdInfo.colliding_obst_index].push_back(v);
  }
#endif
  
  if ( (answerFromEnvironment) && enablePenetration &&
       (m_cd->penetration>=0)) {
    Cfg* tmp = _cfg.CreateNewCfg();
    bool result = !m_cd->AcceptablePenetration(*tmp, env, Stats, _cdInfo);
    delete tmp;
    if( Clear ) 
      delete pCallName;
    { _cfg.SetLabel("VALID",!result); 
      // return result; 
      return !result;}
  }
  if( Clear ) 
    delete pCallName;
  { _cfg.SetLabel("VALID",!answerFromEnvironment); 
    // return answerFromEnvironment; 
    return !answerFromEnvironment;}
}


template<typename CFG>
bool
CollisionDetectionValidity<CFG>::
IsInCollision(Environment* env, StatClass& Stats, CDInfo& _cdInfo, 
	      shared_ptr<MultiBody> lineRobot, bool enablePenetration, std::string *pCallName) 
{  
  int nmulti, robot;
  bool ret_val, collision_found; // needed to go thru ALL obstacles to get ALL info
  CDInfo local_cd_info;
  nmulti = env->GetMultiBodyCount();
  robot = env->GetRobotIndex();
  
  shared_ptr<MultiBody> rob = env->GetMultiBody(robot);
  
  // A line Segment generated on the fly, to check if 'seemingly connectable'.
  /*
    if (lineRobot) {
    rob = lineRobot;
    }
  */
  ret_val = false;
  
  for (int i = 0; i < nmulti; i++) {
    if ( i != robot ) {
      // Note that the below call sets _cdInfo as needed
      collision_found = IsInCollision(env, Stats, _cdInfo, rob, env->GetMultiBody(i), pCallName);
      if ( (collision_found) && ( ! _cdInfo.ret_all_info) ) {
        _cdInfo.colliding_obst_index = i;
        return true;
      } else  if (_cdInfo.ret_all_info) {  // store more info
        if ((collision_found) && (!ret_val)) {
          // colliding_obst_index is always the FIRST obstacle found in collision
          // nearest_obst_index is 'nearest' obstacle (colliding or not)
          local_cd_info.colliding_obst_index = i;
          ret_val = true;
        }

        // Be certain that IsInCollision set _cdInfo.min_dist
        // Check new mins against old, reset *_points if needed
        // Store everything in local_cd_info, copy back to _cdInfo at end of function.

        if (_cdInfo.min_dist < local_cd_info.min_dist) {
          local_cd_info.nearest_obst_index = i;
          local_cd_info.min_dist = _cdInfo.min_dist;
          local_cd_info.robot_point = _cdInfo.robot_point;
          local_cd_info.object_point = _cdInfo.object_point;
        } // end updating local_cd_info
      }
    } else {
      if(ignoreSelfCollision){
        //robot self checking turned off
        if (_cdInfo.ret_all_info) {
          // local_cd_info should contain "all the info" across all objects
          // _cdInfo only contains info for the last one processed above
          _cdInfo = local_cd_info;
        }

        ret_val =false;
      }
      else {

        // robot self checking. Warning: rob and env->GetMultiBody(robot) may NOT be the same.
        if ( (rob->GetBodyCount() > 1) && 
            (IsInCollision(env, Stats, _cdInfo, rob, rob, pCallName)) ) {
          if (_cdInfo.ret_all_info) {
            // set stuff to indicate odd happenning
            _cdInfo.colliding_obst_index = -1;
            _cdInfo.min_dist = MaxDist;
            _cdInfo.nearest_obst_index = -1;
            _cdInfo.robot_point[0] = _cdInfo.robot_point[1] = _cdInfo.robot_point[2] = 0;
            _cdInfo.object_point[0] = _cdInfo.object_point[1] = _cdInfo.object_point[2] = 0;
          }

          return true;

        }
      }
    } // end  if-else i == robot

  } // end for i
  
  if (_cdInfo.ret_all_info) {
    // local_cd_info should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = local_cd_info;
  }
  
  return ret_val;
} // end IsInCollision ( 4 params, 4th defaults to NULL)




template<typename CFG>
bool
CollisionDetectionValidity<CFG>::
IsInCollision(Environment* env, StatClass& Stats, CDInfo& _cdInfo,
    shared_ptr<MultiBody> rob, shared_ptr<MultiBody> obst, std::string *pCallName) {
  std::vector<CollisionDetectionMethod*>::iterator I;
  for(I=m_selected.begin(); I!=m_selected.end(); I++) {
    int tp = (*I)->GetType();
    // Type Out: no collision sure; collision unsure.
    if((tp == Out) && ((*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName, ignore_i_adjacent_links) == false)) {
      return false;
    }

    // Type In: no collision unsure; collision sure.
    if ((tp == In) && ((*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName, ignore_i_adjacent_links) == true)) {
      return true;
    }

    // Type Exact: no collision sure; collision sure.
    if(tp == Exact) {
      return (*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName, ignore_i_adjacent_links);
    }
  }

  return true;
}

template<typename CFG>
bool
CollisionDetectionValidity<CFG>::
isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo)
{
#ifdef USE_PQP
  return Pqp_Solid().isInsideObstacle(cfg, env);
#else
  cerr << "Recompile with PQP to use CollisionDetectionValidity::isInsideObstacle" <<endl;
  exit(1);
#endif
}

#endif // #ifndef _COLLISION_DETECTION_VALIDITY_HPP_
