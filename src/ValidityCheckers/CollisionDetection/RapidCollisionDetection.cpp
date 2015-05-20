#ifdef USE_RAPID

#include "RapidCollisionDetection.h"

#include <RAPID.H>

#include "CDInfo.h"
#include "MPProblem/Geometry/ActiveMultiBody.h"
#include "MPProblem/Geometry/FreeBody.h"

Rapid::
Rapid() : CollisionDetectionMethod("RAPID", CDType::Exact, RAPID) {
}

bool
Rapid::
IsInCollision(shared_ptr<ActiveMultiBody> _robot,
    shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
    size_t _ignoreIAdjacentMultibodies) {

  if(_cdInfo.m_retAllInfo) {
    cerr << endl;
    cerr << "Currently unable to return ALL info using RAPID cd." << endl;
    cerr << "Defaulting to minimal information." << endl;
  }

  for(size_t i = 0; i < _robot->GetFreeBodyCount(); i++) {
    shared_ptr<FreeBody> robotBody = _robot->GetFreeBody(i);
    shared_ptr<RAPID_model> rob = robotBody->GetRapidBody();
    Transformation& t1 = robotBody->WorldTransformation();

    for(size_t j = 0; j < _obstacle->GetBodyCount(); j++) {
      shared_ptr<Body> obstBody = _obstacle->GetBody(j);
      if(_robot == _obstacle){
        //GetBody() first returns fixed bodies, then free bodies.
        //When this body is the same as the one against which we are checking,
        // we know that we have compared with all fixed bodies and any already finished
        // free bodies. In this case, we can stop checking; future free bodies will
        // check themselves against this one.
        if(robotBody == obstBody)
          break;
        //Also, if the two bodies are nearby links, don't compare them.
        if(robotBody->IsWithinI(obstBody, _ignoreIAdjacentMultibodies))
          continue;
      }

      shared_ptr<RAPID_model> obst = obstBody->GetRapidBody();
      Transformation& t2 = obstBody->WorldTransformation();

      if(RAPID_Collide(
            t1.rotation().matrix(), t1.translation(), rob.get(),
            t2.rotation().matrix(), t2.translation(), obst.get(),
            RAPID_FIRST_CONTACT))
        throw RunTimeException(WHERE, "RAPID_ERR_COLLIDE_OUT_OF_MEMORY");

      if(RAPID_num_contacts) {
        _cdInfo.m_rapidContactID1 = RAPID_contact[0].id1;
        _cdInfo.m_rapidContactID2 = RAPID_contact[0].id2;
        return true;
      }
    }
  }

  return false;
}

#endif
