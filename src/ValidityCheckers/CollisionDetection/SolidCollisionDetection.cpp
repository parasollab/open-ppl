#include "SolidCollisionDetection.h"

#ifdef USE_SOLID
Solid::
Solid() : CollisionDetectionMethod() {
  m_name = "SOLID";
  m_type = Exact;
  m_cdtype = SOLID;
}


Solid::
~Solid() {
}

bool
Solid::
IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
	      StatClass& _stats, CDInfo& _cdInfo, std::string *_callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(GetName(), _callName);
 
  _robot->UpdateVertexBase();

  if(_cdInfo.ret_all_info == false){

    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {

      shared_ptr<DT_ObjectHandle> rob = _robot->GetFreeBody(i)->GetSolidBody();

      for(int j=0; j<_obstacle->GetBodyCount(); j++) {

        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
	   _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {
                continue;
        }

        shared_ptr<DT_ObjectHandle> obst = _obstacle->GetBody(j)->GetSolidBody();

        MT_Vector3 cp1,cp2;
        MT_Vector3 separation;
        float tempdist;

        if (DT_GetPenDepth(*rob, *obst, cp1, cp2)){// intersection 
          separation = cp1 - cp2;
          tempdist = cp1.distance(cp2);
          return true;
        }

      } // end for j
    } // end for i

  return false;
  }// _cdInfo.ret_all_info = false
  else{

    bool ret_val = false;
    float dist = MaxDist;  // =  1e10 by CollisionDetection.h
    float tempdist;

    
    // default _cdInfo contents
    _cdInfo.ResetVars();
    _cdInfo.ret_all_info = true;

    for(int i=0 ; i<_robot->GetFreeBodyCount(); i++) {

      shared_ptr<DT_ObjectHandle> rob = _robot->GetFreeBody(i)->GetSolidBody();

      for(int j=0; j<_obstacle->GetBodyCount(); j++) {

        // if robot check self collision, skip adjacent links.
        if(_robot == _obstacle &&
	   _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j),_ignoreIAdjacentMultibodies) ) {
                continue;
        }

        shared_ptr<DT_ObjectHandle> obst = _obstacle->GetBody(j)->GetSolidBody();

        MT_Vector3 cp1,cp2;
        MT_Vector3 separation;

        if (DT_GetPenDepth(*rob, *obst, cp1, cp2)){// intersection 
          separation = cp1 - cp2;
          tempdist = -cp1.distance(cp2);

	  DT_GetPenDepth(*obst, *rob, cp1, cp2);
          separation = cp1 - cp2;
          tempdist = -cp1.distance(cp2);

          ret_val = true;
          if(tempdist < dist){
            dist = tempdist;
            _cdInfo.colliding_obst_index = j;
            _cdInfo.nearest_obst_index = j;
            _cdInfo.min_dist = dist;


            _cdInfo.robot_point = _robot->GetFreeBody(i)->WorldTransformation() * Vector3d(cp1[0],cp1[1],cp1[2]);
            _cdInfo.object_point = _obstacle->GetBody(j)->WorldTransformation() * Vector3d(cp2[0],cp2[1],cp2[2]);
          }

        }else{// no intersection
          DT_GetClosestPair(*rob, *obst, cp1, cp2);
          separation = cp2 - cp1;
          tempdist = cp1.distance(cp2);
          if(tempdist < dist){
            dist = tempdist;
            _cdInfo.nearest_obst_index = j;
	    if(dist == 0){
	      ret_val = true;
	      _cdInfo.colliding_obst_index = j;
	    }
            _cdInfo.min_dist = dist;
            _cdInfo.robot_point = _robot->GetFreeBody(i)->WorldTransformation() * Vector3d(cp1[0],cp1[1],cp1[2]);
            _cdInfo.object_point = _obstacle->GetBody(j)->WorldTransformation() * Vector3d(cp2[0],cp2[1],cp2[2]);
          }
        }

      } // end for j
    } // end for i

    return ret_val;
  }// _cdInfo.ret_all_info = true

} // end IsInCollision_solid()


#endif

