#include "RapidCollisionDetection.h"

#include "CDInfo.h"
#include "Geometry/Bodies/Body.h"

#include <RAPID.H>


/*------------------------------- Construction -------------------------------*/

Rapid::
Rapid() : CollisionDetectionMethod("RAPID") { }

/*------------------------------- CD Interface -------------------------------*/

void
Rapid::
Build(Body* const _body) {
  const GMSPolyhedron& poly = _body->GetPolyhedron();
  RAPID_model* rapidBody = new RAPID_model;
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
IsInCollision(const Body* const _body1, const Body* const _body2,
    CDInfo& _cdInfo) {

  auto body1 = _body1->GetRapidBody();
  auto body2 = _body2->GetRapidBody();
  /// @TODO See if we can avoid the copy here with a const cast.
  //const Transformation& t1 = _body1->GetWorldTransformation();
  //const Transformation& t2 = _body2->GetWorldTransformation();
  Transformation t1 = _body1->GetWorldTransformation();
  Transformation t2 = _body2->GetWorldTransformation();

  const int flag = _cdInfo.m_retAllInfo ? RAPID_ALL_CONTACTS
                                        : RAPID_FIRST_CONTACT;

  if(RAPID_Collide(
        t1.rotation().matrix(), t1.translation(), body1,
        t2.rotation().matrix(), t2.translation(), body2,
        flag))
  {
    throw RunTimeException(WHERE, "RAPID_ERR_COLLIDE_OUT_OF_MEMORY");
  }

  for(int i = 0; i < RAPID_num_contacts; ++i)
    _cdInfo.m_trianglePairs.emplace_back(RAPID_contact[i].id1,
                                         RAPID_contact[i].id2);

  return RAPID_num_contacts;
}

/*----------------------------------------------------------------------------*/
