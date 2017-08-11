#include "RapidCollisionDetection.h"

#ifndef NO_RAPID

#include <RAPID.H>

#include "CDInfo.h"
#include "Geometry/Bodies/Body.h"


/*------------------------------- Construction -------------------------------*/

Rapid::
Rapid() : CollisionDetectionMethod("RAPID") { }

/*------------------------------- CD Interface -------------------------------*/

void
Rapid::
Build(Body* const _body) {
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  unique_ptr<RAPID_model> rapidBody(new RAPID_model);
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
  _body->SetRapidBody(move(rapidBody));
}


bool
Rapid::
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {

  auto body1 = _body1->GetRapidBody();
  auto body2 = _body2->GetRapidBody();
  /// @TODO See if we can modify RAPID_Collide to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  //const Transformation& t1 = _body1->GetWorldTransformation();
  //const Transformation& t2 = _body2->GetWorldTransformation();
  Transformation t1 = _body1->GetWorldTransformation();
  Transformation t2 = _body2->GetWorldTransformation();

  if(RAPID_Collide(
        t1.rotation().matrix(), t1.translation(), body1,
        t2.rotation().matrix(), t2.translation(), body2,
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
