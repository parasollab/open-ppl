#include "RapidCollisionDetection.h"

#ifndef NO_RAPID

#include <RAPID.H>

#include "CDInfo.h"
#include "Environment/Body.h"


/*------------------------------- Construction -------------------------------*/

Rapid::
Rapid() : CollisionDetectionMethod("RAPID") { }

/*------------------------------- CD Interface -------------------------------*/

void
Rapid::
Build(Body* _body) {
  GMSPolyhedron& poly = _body->GetPolyhedron();
  shared_ptr<RAPID_model> rapidBody(new RAPID_model);
  rapidBody->BeginModel();
  for(size_t q = 0; q < poly.m_polygonList.size(); q++) {
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      const Vector3d& tmp = poly.m_polygonList[q].GetPoint(i);
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    rapidBody->AddTri(point[0], point[1], point[2], q);
  }
  rapidBody->EndModel();
  _body->SetRapidBody(rapidBody);
}


bool
Rapid::
IsInCollision(shared_ptr<Body> _body1, shared_ptr<Body> _body2,
    CDInfo& _cdInfo) {

  shared_ptr<RAPID_model> body1 = _body1->GetRapidBody();
  shared_ptr<RAPID_model> body2 = _body2->GetRapidBody();
  Transformation& t1 = _body1->WorldTransformation();
  Transformation& t2 = _body2->WorldTransformation();

  if(RAPID_Collide(
        t1.rotation().matrix(), t1.translation(), body1.get(),
        t2.rotation().matrix(), t2.translation(), body2.get(),
        RAPID_FIRST_CONTACT))
    throw RunTimeException(WHERE, "RAPID_ERR_COLLIDE_OUT_OF_MEMORY");

  if(RAPID_num_contacts) {
    _cdInfo.m_rapidContactID1 = RAPID_contact[0].id1;
    _cdInfo.m_rapidContactID2 = RAPID_contact[0].id2;
    return true;
  }

  return false;
}

/*----------------------------------------------------------------------------*/

#endif
