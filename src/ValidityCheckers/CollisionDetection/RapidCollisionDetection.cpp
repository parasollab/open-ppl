#ifdef USE_RAPID

#include "RapidCollisionDetection.h"
#include "Utilities/MetricUtils.h"
#include "MPProblem/Geometry/MultiBody.h"
#include <RAPID.H>
#include "CDInfo.h"

Rapid::Rapid() : CollisionDetectionMethod("RAPID", Exact, RAPID) {}

Rapid::~Rapid() {}

bool
Rapid::IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
    StatClass& _stats, CDInfo& _cdInfo, string* _callName, int _ignoreIAdjacentMultibodies) {
  _stats.IncNumCollDetCalls(m_name, _callName);

  if (_cdInfo.m_retAllInfo) {
    cerr << endl;
    cerr << "Currently unable to return ALL info using RAPID cd." << endl;
    cerr << "Defaulting to minimal information." << endl;
  }

  for(int i=0 ; i<_robot->GetFreeBodyCount(); i++){
    shared_ptr<RAPID_model> rob = _robot->GetFreeBody(i)->GetRapidBody();

    for(int j=0; j<_obstacle->GetBodyCount(); j++){
      if(_robot == _obstacle &&
          _robot->GetFreeBody(i)->IsWithinI(_obstacle->GetBody(j), _ignoreIAdjacentMultibodies) ){
        continue;
      }

      shared_ptr<RAPID_model> obst = _obstacle->GetBody(j)->GetRapidBody();
      Transformation& t1 = _robot->GetFreeBody(i)->WorldTransformation();
      Transformation& t2 = _obstacle->GetBody(j)->WorldTransformation();

      if(RAPID_Collide(t1.rotation().matrix(), t1.translation(), rob.get(),
            t2.rotation().matrix(), t2.translation(), obst.get(), RAPID_FIRST_CONTACT)) {
        cerr << "Error in CollisionDetection::RAPID_Collide, RAPID_ERR_COLLIDE_OUT_OF_MEMORY" << endl;
        exit(1);
      }

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

